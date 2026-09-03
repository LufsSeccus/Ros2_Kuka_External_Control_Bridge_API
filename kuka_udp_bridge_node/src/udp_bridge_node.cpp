#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>   
#include <iomanip>   
#include <algorithm> 

// Linux Socket Headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

// ROS2 Headers
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class KukaUdpBridge : public rclcpp::Node {
public:
    KukaUdpBridge() : Node("kuka_udp_bridge"), tx_counter_(0), last_base_reached_state_(true) {
<<<<<<< HEAD
        this->declare_parameter<std::string>("network_interface", "");
=======
        this->declare_parameter<std::string>("network_interface", "eth0");
>>>>>>> 51927bc3690c9f62b054033efb1204bdd45885fe
        network_interface_ = this->get_parameter("network_interface").as_string();
        this->declare_parameter<std::string>("robot_ip", "172.31.1.10");
        this->declare_parameter<int>("robot_port", 30300);
        this->declare_parameter<int>("client_port", 30333);

        robot_ip_ = this->get_parameter("robot_ip").as_string();
        robot_port_ = this->get_parameter("robot_port").as_int();
        client_port_ = this->get_parameter("client_port").as_int();

        RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Target Robot: %s:%d", robot_ip_.c_str(), robot_port_);
        RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Local Listening Port: %d", client_port_);

        setup_sockets();

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        base_target_pub_ = this->create_publisher<std_msgs::msg::Bool>("base_target_reached", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&KukaUdpBridge::cmd_vel_callback, this, std::placeholders::_1));

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&KukaUdpBridge::goal_pose_callback, this, std::placeholders::_1));

        arm_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "arm_cmd_joints", 10, std::bind(&KukaUdpBridge::arm_joint_callback, this, std::placeholders::_1));

        arm_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "arm_goal_pose", 10, std::bind(&KukaUdpBridge::arm_pose_callback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        rx_thread_active_ = true;
        rx_thread_ = std::thread(&KukaUdpBridge::receive_thread_loop, this);

        send_to_robot("App_Start", "true");
        RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Sent App_Start handshake to robot.");
    }

    ~KukaUdpBridge() {
        RCLCPP_WARN(this->get_logger(), "[KUKA UDP Bridge] Shutting down bridge node...");
        send_to_robot("Set_Shutdown", "true");
        rx_thread_active_ = false;

        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (sock_fd_ >= 0) {
            close(sock_fd_);
        }
        if (telemetry_log_file_.is_open()) {
            telemetry_log_file_.close();
            RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Telemetry log file saved and closed.");
        }
    }

private:
    void setup_sockets() {
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket!");
            throw std::runtime_error("Socket creation failed");
        }
        // --- NEW: Hardware-level socket binding ---
        if (!network_interface_.empty()) {
            if (setsockopt(sock_fd_, SOL_SOCKET, SO_BINDTODEVICE, network_interface_.c_str(), network_interface_.length()) < 0) {
                RCLCPP_ERROR(this->get_logger(), 
                    "Failed to bind to hardware interface %s. Did you run the setcap command?", 
                    network_interface_.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), 
                    ">>> Socket hardware-bound strictly to interface: %s <<<", 
                    network_interface_.c_str());
            }
        }
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(client_port_);

        if (bind(sock_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind UDP socket to port %d!", client_port_);
            throw std::runtime_error("Socket bind failed");
        }

        memset(&robot_addr_, 0, sizeof(robot_addr_));
        robot_addr_.sin_family = AF_INET;
        robot_addr_.sin_port = htons(robot_port_);
        inet_pton(AF_INET, robot_ip_.c_str(), &robot_addr_.sin_addr);
    }

    void send_to_robot(const std::string& command, const std::string& value) {
        tx_counter_++;
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        std::stringstream ss;
        ss << ms << ";" << tx_counter_ << ";" << command << ";" << value;
        std::string payload = ss.str();

        sendto(sock_fd_, payload.c_str(), payload.length(), 0,
               (struct sockaddr *)&robot_addr_, sizeof(robot_addr_));
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::stringstream ss;
        ss << msg->linear.x << "," << msg->linear.y << "," << msg->angular.z;
        send_to_robot("Set_Vel", ss.str());
    }

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double x_mm = msg->pose.position.x * 1000.0;
        double y_mm = msg->pose.position.y * 1000.0;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double alpha_deg = yaw * (180.0 / M_PI);

        std::stringstream ss;
        ss << x_mm << "," << y_mm << "," << alpha_deg;
        send_to_robot("Set_Pose", ss.str());
    }

    void arm_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 7) return;
        std::stringstream ss;
        for (size_t i = 0; i < 7; ++i) {
            ss << (msg->position[i] * (180.0 / M_PI));
            if (i < 6) ss << ",";
        }
        send_to_robot("Set_Arm_Joint", ss.str());
    }

    void arm_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double x_m = msg->pose.position.x;
        double y_m = msg->pose.position.y;
        double z_m = msg->pose.position.z;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        double roll_rad, pitch_rad, yaw_rad;
        tf2::Matrix3x3(q).getRPY(roll_rad, pitch_rad, yaw_rad);

        std::stringstream ss;
        ss << x_m << "," << y_m << "," << z_m << ","
           << yaw_rad << "," << pitch_rad << "," << roll_rad;

        send_to_robot("Set_Arm_End_Effector", ss.str());
    }

    std::string get_log_filename() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm parts;
        localtime_r(&now_c, &parts);

        std::ostringstream oss;
        oss << "kuka_log/logger_bridge_file_" 
            << std::put_time(&parts, "%H-%M_%d-%m-%Y") 
            << ".csv";
        return oss.str();
    }

    std::string format_raw_timestamp(const std::string& raw_ts_str) {
        try {
            uint64_t raw_ms = std::stoull(raw_ts_str);
            std::time_t sec = static_cast<std::time_t>(raw_ms / 1000);
            uint32_t remainder_ms = static_cast<uint32_t>(raw_ms % 1000);

            std::tm parts;
            localtime_r(&sec, &parts);

            std::ostringstream oss;
            oss << std::put_time(&parts, "%Y-%m-%d %H:%M:%S")
                << "." << std::setfill('0') << std::setw(3) << remainder_ms;
            return oss.str();
        } catch (...) {
            return raw_ts_str;
        }
    }

    void receive_thread_loop() {
        char rx_buf[1024];
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        fcntl(sock_fd_, F_SETFL, O_NONBLOCK);
        RCLCPP_INFO(this->get_logger(), "[KUKA UDP RX] Thread spawned. Listening for status payloads...");

        while (rx_thread_active_ && rclcpp::ok()) {
            memset(rx_buf, 0, sizeof(rx_buf));
            int bytes_received = recvfrom(sock_fd_, rx_buf, sizeof(rx_buf) - 1, 0,
                                          (struct sockaddr *)&sender_addr, &sender_len);

            if (bytes_received > 0) {
                rx_buf[bytes_received] = '\0';
                std::string msg(rx_buf);

                if (!is_logging_started_) {
                    std::string filename = get_log_filename();
                    telemetry_log_file_.open(filename, std::ios::out | std::ios::app);

                    if (telemetry_log_file_.is_open()) {
                        telemetry_log_file_ << "Timestamp,ErrorCode,Counter,"
                                            << "KMP_X,KMP_Y,KMP_Alpha,BaseTargetReached,"
                                            << "Arm_J1,Arm_J2,Arm_J3,Arm_J4,Arm_J5,Arm_J6,Arm_J7\n";
                        is_logging_started_ = true;
                    }
                }

                if (is_logging_started_ && telemetry_log_file_.is_open()) {
                    std::string csv_line = msg;
                    std::replace(csv_line.begin(), csv_line.end(), ';', ',');
                    size_t first_comma = csv_line.find(',');
                    if (first_comma != std::string::npos) {
                        std::string raw_ts = csv_line.substr(0, first_comma);
                        std::string readable_ts = format_raw_timestamp(raw_ts);
                        csv_line = readable_ts + csv_line.substr(first_comma);
                    }
                    telemetry_log_file_ << csv_line << "\n";
                    telemetry_log_file_.flush();
                }

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "[KUKA Telemetry RX] Raw Payload: %s", msg.c_str());

                parse_and_publish_telemetry(msg);
            } else {
                std::this_thread::sleep_for(2ms);
            }
        }
    }

    void parse_and_publish_telemetry(const std::string& msg) {
        std::vector<std::string> parts;
        std::stringstream ss(msg);
        std::string item;
        while (std::getline(ss, item, ';')) {
            parts.push_back(item);
        }

        if (parts.size() < 4) return;

        try {
            // 1. Base Pose & Flag Processing from parts[3]
            // Format: X,Y,Alpha,baseTargetReached
            std::vector<double> base_data;
            std::stringstream base_ss(parts[3]);
            std::string val;
            while (std::getline(base_ss, val, ',')) {
                base_data.push_back(std::stod(val));
            }

            if (base_data.size() >= 3) {
                double x_m = base_data[0] / 1000.0;
                double y_m = base_data[1] / 1000.0;
                double yaw_rad = base_data[2] *(M_PI / 180.0);

                // Publish Odometry
                auto odom_msg = nav_msgs::msg::Odometry();
                odom_msg.header.stamp = this->now();
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "base_footprint";
                odom_msg.pose.pose.position.x = x_m;
                odom_msg.pose.pose.position.y = y_m;

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw_rad);
                odom_msg.pose.pose.orientation.x = q.x();
                odom_msg.pose.pose.orientation.y = q.y();
                odom_msg.pose.pose.orientation.z = q.z();
                odom_msg.pose.pose.orientation.w = q.w();
                odom_pub_->publish(odom_msg);

                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header = odom_msg.header;
                tf_msg.child_frame_id = odom_msg.child_frame_id;
                tf_msg.transform.translation.x = x_m;
                tf_msg.transform.translation.y = y_m;
                tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
                tf_broadcaster_->sendTransform(tf_msg);
            }

            // Extract baseTargetReached if it's the 4th comma-separated value in base_data (index 3)
            if (base_data.size() >= 4) {
                bool current_reached_state = (base_data[3] >= 0.5); // 1.0 means reached, 0.0 means moving

                if (current_reached_state != last_base_reached_state_) {
                    RCLCPP_INFO(this->get_logger(), 
                        ">>> [KMP STATE CHANGE] Target Reached Flag: %s <<<", 
                        current_reached_state ? "YES (1 - Reached)" : "NO (0 - Moving)");
                    last_base_reached_state_ = current_reached_state;
                }

                auto reached_msg = std_msgs::msg::Bool();
                reached_msg.data = current_reached_state; 
                base_target_pub_->publish(reached_msg);
            }

            // 2. LBR Joint State Processing (Parts[4] is the arm payload)
            if (parts.size() >= 5) {
                std::vector<double> arm_joints_deg;
                std::stringstream arm_ss(parts[4]);
                while (std::getline(arm_ss, val, ',')) {
                    arm_joints_deg.push_back(std::stod(val));
                }

                if (arm_joints_deg.size() >= 7) {
                    auto joint_msg = sensor_msgs::msg::JointState();
                    joint_msg.header.stamp = this->now();
                    joint_msg.name = {
                        "lbr_joint_1", "lbr_joint_2", "lbr_joint_3",
                        "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"
                    };

                    for (int i = 0; i < 7; ++i) {
                        joint_msg.position.push_back(arm_joints_deg[i] * (M_PI / 180.0));
                    }
                    joint_pub_->publish(joint_msg);
                }
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "[KUKA Telemetry Parse Error] %s", e.what());
        }
    }
    int sock_fd_ = -1;
    struct sockaddr_in robot_addr_;
    std::string robot_ip_;
    std::string network_interface_;
    int robot_port_;
    int client_port_;
    long tx_counter_;

    std::ofstream telemetry_log_file_;
    bool is_logging_started_ = false;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr base_target_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::thread rx_thread_;
    std::atomic<bool> rx_thread_active_;
    bool last_base_reached_state_; 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KukaUdpBridge>());
    rclcpp::shutdown();
    return 0;
}