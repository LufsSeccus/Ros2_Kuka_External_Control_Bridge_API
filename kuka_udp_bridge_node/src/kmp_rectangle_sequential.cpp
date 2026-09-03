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

class KmpRectangleSequential : public rclcpp::Node {
public:
    KmpRectangleSequential() : Node("kmp_rectangle_sequential"), current_step_(0), current_state_(State::WAITING_FOR_INIT) {
        goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        arm_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("arm_cmd_joints", 10);

        base_target_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "base_target_reached", 10, std::bind(&KmpRectangleSequential::baseReachedCallback, this, std::placeholders::_1));
            
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&KmpRectangleSequential::jointStateCallback, this, std::placeholders::_1));

        waypoints_ = {
            {1.0, 0.0, 0.0,   {0.0, 20.0, 0.0, -90.0, 0.0, 60.0, 0.0}},
            {1.0, 1.0, 0.0,   {0.0, 45.0, 0.0, -60.0, 0.0, 45.0, 0.0}},
            {0.0, 1.0, 0.0,   {30.0, 30.0, 0.0, -75.0, 0.0, 30.0, 0.0}},
            {0.0, 0.0, 0.0,   {0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0}}
        };

        // 1-second startup timer to begin execution sequence
        init_timer_ = this->create_wall_timer(
            1s, [this]() {
                init_timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "=== Starting Sequential Trajectory Execution ===");
                current_state_ = State::MOVING_ARM;
                publishArmWaypoint(current_step_);
            });
    }

private:
    enum class State {
        WAITING_FOR_INIT,
        MOVING_ARM,
        MOVING_BASE,
        COMPLETED
    };

    void publishArmWaypoint(size_t index) {
        if (index >= waypoints_.size()) return;
        const auto &wp = waypoints_[index];

        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        joint_msg.name = {"lbr_joint_1", "lbr_joint_2", "lbr_joint_3", "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"};
        for (double deg : wp.arm_joints_deg) {
            joint_msg.position.push_back(deg * (M_PI / 180.0));
        }
        arm_joint_pub_->publish(joint_msg);

        dispatch_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "[Step %zu/%zu] Moving ARM to target...", index + 1, waypoints_.size());
    }

    void publishBaseWaypoint(size_t index) {
        if (index >= waypoints_.size()) return;
        const auto &wp = waypoints_[index];

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

        dispatch_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "[Step %zu/%zu] Arm finished. Moving BASE to (%.2fm, %.2fm)...", 
            index + 1, waypoints_.size(), wp.base_x, wp.base_y);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (current_state_ != State::MOVING_ARM || current_step_ >= waypoints_.size()) return;
        if (msg->position.size() < 7) return;

        // Masking constraint: Ignore joint checks for the first 0.5 seconds to bypass residual readings
        if ((this->now() - dispatch_time_).seconds() < 0.5) return;

        const auto &target = waypoints_[current_step_];
        double max_joint_error_rad = 0.0;
        
        for (size_t i = 0; i < 7; ++i) {
            double err = std::abs((target.arm_joints_deg[i] * (M_PI / 180.0)) - msg->position[i]);
            if (err > max_joint_error_rad) max_joint_error_rad = err;
        }

        // Tolerance limit: ~2 degrees
        if (max_joint_error_rad < 0.035) {
            RCLCPP_INFO(this->get_logger(), "[Status] Arm reached target for step %zu.", current_step_ + 1);
            
            // Switch state to moving base next
            current_state_ = State::MOVING_BASE;
            publishBaseWaypoint(current_step_);
        }
    }

    void baseReachedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (current_state_ != State::MOVING_BASE || current_step_ >= waypoints_.size()) return;

        bool is_at_target = msg->data;
        double elapsed_seconds = (this->now() - dispatch_time_).seconds();

        // Masking constraint: Ignore '1' for the first 0.5 seconds to bypass previous waypoint leftovers
        if (is_at_target && elapsed_seconds > 0.5) {
            RCLCPP_INFO(this->get_logger(), "[Status] Base reached target for step %zu.", current_step_ + 1);
            
            current_step_++;
            if (current_step_ < waypoints_.size()) {
                // Proceed to next waypoint, starting with the arm again
                current_state_ = State::MOVING_ARM;
                publishArmWaypoint(current_step_);
            } else {
                current_state_ = State::COMPLETED;
                RCLCPP_INFO(this->get_logger(), "=== Full Sequential Trajectory Complete! ===");
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_step_;
    State current_state_;
    rclcpp::Time dispatch_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KmpRectangleSequential>());
    rclcpp::shutdown();
    return 0;
}