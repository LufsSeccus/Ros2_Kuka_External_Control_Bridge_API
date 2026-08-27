#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

struct Waypoint {
    double base_x;
    double base_y;
    double base_yaw_deg;
    std::vector<double> arm_joints_deg;
};

class KmpRectangleNode : public rclcpp::Node {
public:
    KmpRectangleNode() : Node("kmp_rectangle_node"), current_step_(0), is_waiting_(false) {
        goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        arm_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_cmd_joints", 10);

        base_idle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/base_idle", 10, std::bind(&KmpRectangleNode::baseIdleCallback, this, std::placeholders::_1));
            
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&KmpRectangleNode::jointStateCallback, this, std::placeholders::_1));

        waypoints_ = {
            {1.0, 0.0, 0.0,   {0.0, 20.0, 0.0, -90.0, 0.0, 60.0, 0.0}},
            {1.0, 1.0, 0.0,  {0.0, 45.0, 0.0, -60.0, 0.0, 45.0, 0.0}},
            {0.0, 1.0, 0.0, {30.0, 30.0, 0.0, -75.0, 0.0, 30.0, 0.0}},
            {0.0, 0.0, 0.0,   {0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0}}
        };

        init_timer_ = this->create_wall_timer(
            1s, [this]() {
                init_timer_->cancel();
                publishWaypoint(current_step_);
            });
    }

private:
    void publishWaypoint(size_t index) {
        if (index >= waypoints_.size()) return;
        const auto &wp = waypoints_[index];

        // 1. Reset synchronization flags and start the delay mask timer
        base_reached_ = false;
        arm_reached_ = false;
        is_waiting_ = true;
        dispatch_time_ = this->now();

        // 2. Publish Base Goal
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.position.x = wp.base_x;
        pose_msg.pose.position.y = wp.base_y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, wp.base_yaw_deg * (M_PI / 180.0));
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        goal_pose_pub_->publish(pose_msg);

        // 3. Publish Arm Goal
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        joint_msg.name = {"lbr_joint_1", "lbr_joint_2", "lbr_joint_3", "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"};
        for (double deg : wp.arm_joints_deg) {
            joint_msg.position.push_back(deg * (M_PI / 180.0));
        }
        arm_joint_pub_->publish(joint_msg);

        RCLCPP_INFO(this->get_logger(),
            "[Dispatched] WP [%zu/%zu] -> Base: (%.2fm, %.2fm, %.1f°) | Arm J1: %.1f°",
            index + 1, waypoints_.size(), wp.base_x, wp.base_y, wp.base_yaw_deg, wp.arm_joints_deg[0]);
    }

    void baseReachedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!is_waiting_ || base_reached_ || current_step_ >= waypoints_.size()) return;

        bool is_at_target = msg->data;

        if (!is_at_target) {
            // Java set the flag to 0 (Command entered queue)
            base_acknowledged_ = true;
        } 
        else if (is_at_target && base_acknowledged_) {
            // Java set the flag back to 1 (kmp.move() unblocked & queue empty)
            base_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "[Status] Base arrived at WP %zu (Java kmp.move() finished).", current_step_ + 1);
            checkAndProceed();
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!is_waiting_ || arm_reached_ || current_step_ >= waypoints_.size()) return;
        if (msg->position.size() < 7) return;

        const auto &target = waypoints_[current_step_];
        double max_joint_error_rad = 0.0;
        
        for (size_t i = 0; i < 7; ++i) {
            double err = std::abs((target.arm_joints_deg[i] * (M_PI / 180.0)) - msg->position[i]);
            if (err > max_joint_error_rad) max_joint_error_rad = err;
        }

        if (max_joint_error_rad < 0.035) { // Tolerance ~2 degrees
            arm_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "[Status] Arm arrived at WP %zu.", current_step_ + 1);
            checkAndProceed();
        }
    }

    void checkAndProceed() {
        if (base_reached_ && arm_reached_) {
            RCLCPP_INFO(this->get_logger(), ">> Both Base and Arm reached WP %zu. Proceeding... <<", current_step_ + 1);
            is_waiting_ = false;
            current_step_++;

            if (current_step_ < waypoints_.size()) {
                publishWaypoint(current_step_);
            } else {
                RCLCPP_INFO(this->get_logger(), "=== Full Trajectory Complete! ===");
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_idle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_step_;
    
    rclcpp::Time dispatch_time_;
    bool is_waiting_;
    bool base_reached_;
    bool arm_reached_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KmpRectangleNode>());
    rclcpp::shutdown();
    return 0;
}