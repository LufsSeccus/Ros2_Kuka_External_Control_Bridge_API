#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <random>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class BadmintonDemo : public rclcpp::Node {
public:
    BadmintonDemo() : Node("badminton_demo"), current_turn_(1) {
        // Robot 1 Publishers & Subscribers
        goal_pub_1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot1/goal_pose", 10);
        arm_pub_1_  = this->create_publisher<sensor_msgs::msg::JointState>("/robot1/arm_cmd_joints", 10);
        base_sub_1_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot1/base_target_reached", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { baseCallback(1, msg); });
        joint_sub_1_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/robot1/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointCallback(1, msg); });

        // Robot 2 Publishers & Subscribers
        goal_pub_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);
        arm_pub_2_  = this->create_publisher<sensor_msgs::msg::JointState>("/robot2/arm_cmd_joints", 10);
        base_sub_2_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot2/base_target_reached", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { baseCallback(2, msg); });
        joint_sub_2_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/robot2/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointCallback(2, msg); });

        // Arm States (Degrees)
        arm_states_ = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},       // 0: HOMING
            {0.0, 30.0, 0.0, -110.0, 0.0, 45.0, 0.0},    // 1: UNDERNEATH FLICK
            {0.0, -30.0, 0.0, -60.0, 0.0, -45.0, 0.0},   // 2: TOP FLICK
            {-45.0, 20.0, 0.0, -80.0, 45.0, 30.0, 0.0},  // 3: RIGHT FLICK
            {45.0, 20.0, 0.0, -80.0, -45.0, 30.0, 0.0}   // 4: LEFT FLICK
        };

        // Initialize random generators
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator_ = std::mt19937(seed);
        arm_dist_ = std::uniform_int_distribution<int>(1, 4);        // Exclude 0 (Homing)
        pos_dist_ = std::uniform_real_distribution<double>(-1.0, 1.0); // max rand is 1.0
        yaw_dist_ = std::uniform_real_distribution<double>(-30.0, 30.0);

        // Start Demo Loop
        init_timer_ = this->create_wall_timer(2s, [this]() {
            init_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "=== BADMINTON DEMO STARTING ===");
            dispatchRobot(1);
        });
    }

    ~BadmintonDemo() {
        RCLCPP_WARN(this->get_logger(), "Node destroyed! Forcing both arms to HOMING state...");
        
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.name = {"lbr_joint_1", "lbr_joint_2", "lbr_joint_3", "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"};
        for (double deg : arm_states_[0]) {
            joint_msg.position.push_back(deg * (M_PI / 180.0));
        }

        // Publish to both directly before destruction
        if (arm_pub_1_) arm_pub_1_->publish(joint_msg);
        if (arm_pub_2_) arm_pub_2_->publish(joint_msg);

        // Brief sleep to ensure network packets are sent before process dies
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

private:
    void dispatchRobot(int id) {
        // 1. Pick Random States
        int random_arm_idx = arm_dist_(generator_);
        std::vector<double> target_joints = arm_states_[random_arm_idx];
        
        double dx = pos_dist_(generator_);
        double dy = pos_dist_(generator_);
        double dyaw = yaw_dist_(generator_);

        // 2. Assign and Publish to the specific robot
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "odom";

        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        joint_msg.name = {"lbr_joint_1", "lbr_joint_2", "lbr_joint_3", "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"};
        for (double deg : target_joints) {
            joint_msg.position.push_back(deg * (M_PI / 180.0));
        }

        tf2::Quaternion q;
        if (id == 1) {
            r1_x_ += dx; r1_y_ += dy; r1_yaw_ += dyaw;
            pose_msg.pose.position.x = r1_x_;
            pose_msg.pose.position.y = r1_y_;
            q.setRPY(0.0, 0.0, r1_yaw_ * (M_PI / 180.0));
            pose_msg.pose.orientation.x = q.x(); pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z(); pose_msg.pose.orientation.w = q.w();

            r1_target_joints_ = target_joints;
            r1_base_reached_ = false;
            r1_arm_reached_ = false;
            dispatch_time_1_ = this->now();

            goal_pub_1_->publish(pose_msg);
            arm_pub_1_->publish(joint_msg);
        } else {
            r2_x_ += dx; r2_y_ += dy; r2_yaw_ += dyaw;
            pose_msg.pose.position.x = r2_x_;
            pose_msg.pose.position.y = r2_y_;
            q.setRPY(0.0, 0.0, r2_yaw_ * (M_PI / 180.0));
            pose_msg.pose.orientation.x = q.x(); pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z(); pose_msg.pose.orientation.w = q.w();

            r2_target_joints_ = target_joints;
            r2_base_reached_ = false;
            r2_arm_reached_ = false;
            dispatch_time_2_ = this->now();

            goal_pub_2_->publish(pose_msg);
            arm_pub_2_->publish(joint_msg);
        }

        RCLCPP_INFO(this->get_logger(), "[Robot %d] Dispatched: Arm State %d | Base Offset: X%+0.2f, Y%+0.2f, Yaw%+0.2f deg", 
                    id, random_arm_idx, dx, dy, dyaw);
    }

    void checkProgress(int id) {
        if (id == 1 && current_turn_ == 1 && r1_base_reached_ && r1_arm_reached_) {
            RCLCPP_INFO(this->get_logger(), "[Robot 1] Finished sequence. Handing over to Robot 2.");
            current_turn_ = 2;
            dispatchRobot(2);
        } 
        else if (id == 2 && current_turn_ == 2 && r2_base_reached_ && r2_arm_reached_) {
            RCLCPP_INFO(this->get_logger(), "[Robot 2] Finished sequence. Handing over to Robot 1.");
            current_turn_ = 1;
            dispatchRobot(1);
        }
    }

    void baseCallback(int id, const std_msgs::msg::Bool::SharedPtr msg) {
        if (current_turn_ != id) return;
        
        auto dispatch_time = (id == 1) ? dispatch_time_1_ : dispatch_time_2_;
        if ((this->now() - dispatch_time).seconds() < 0.5) return; // Mask early triggers

        if (msg->data) {
            if (id == 1 && !r1_base_reached_) {
                r1_base_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "[Robot 1] Base Reached.");
                checkProgress(1);
            } else if (id == 2 && !r2_base_reached_) {
                r2_base_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "[Robot 2] Base Reached.");
                checkProgress(2);
            }
        }
    }

    void jointCallback(int id, const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (current_turn_ != id || msg->position.size() < 7) return;

        auto dispatch_time = (id == 1) ? dispatch_time_1_ : dispatch_time_2_;
        if ((this->now() - dispatch_time).seconds() < 0.5) return; // Mask early triggers

        const auto& target = (id == 1) ? r1_target_joints_ : r2_target_joints_;
        double max_err = 0.0;
        for (size_t i = 0; i < 7; ++i) {
            double err = std::abs((target[i] * (M_PI / 180.0)) - msg->position[i]);
            if (err > max_err) max_err = err;
        }

        if (max_err < 0.035) { // Roughly 2 degrees tolerance
            if (id == 1 && !r1_arm_reached_) {
                r1_arm_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "[Robot 1] Arm Reached.");
                checkProgress(1);
            } else if (id == 2 && !r2_arm_reached_) {
                r2_arm_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "[Robot 2] Arm Reached.");
                checkProgress(2);
            }
        }
    }

    // State Variables
    int current_turn_;
    std::vector<std::vector<double>> arm_states_;
    
    // Robot 1 Tracking
    double r1_x_ = 0.0, r1_y_ = 0.0, r1_yaw_ = 0.0;
    std::vector<double> r1_target_joints_;
    bool r1_base_reached_ = false, r1_arm_reached_ = false;
    rclcpp::Time dispatch_time_1_;

    // Robot 2 Tracking
    double r2_x_ = 0.0, r2_y_ = 0.0, r2_yaw_ = 0.0;
    std::vector<double> r2_target_joints_;
    bool r2_base_reached_ = false, r2_arm_reached_ = false;
    rclcpp::Time dispatch_time_2_;

    // Random Generators
    std::mt19937 generator_;
    std::uniform_int_distribution<int> arm_dist_;
    std::uniform_real_distribution<double> pos_dist_;
    std::uniform_real_distribution<double> yaw_dist_;

    // ROS 2 Objects
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_1_, goal_pub_2_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_pub_1_, arm_pub_2_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_sub_1_, base_sub_2_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_1_, joint_sub_2_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BadmintonDemo>());
    rclcpp::shutdown();
    return 0;
}