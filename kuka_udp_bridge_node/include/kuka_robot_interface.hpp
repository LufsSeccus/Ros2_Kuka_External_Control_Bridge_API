#ifndef KUKA_ROBOT_INTERFACE_HPP_
#define KUKA_ROBOT_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"

class KukaRobotInterface {
public:
    KukaRobotInterface(rclcpp::Node* node_ptr, int robot_id, const std::vector<double>& default_joints);

    // Command API
    void commandBase(double x, double y, double yaw_deg);
    void commandArm(const std::vector<double>& target_deg);
    void setSpeed(double arm_pct, double base_pct);

    // State Tracking API
    bool isBaseReached() const { return base_reached_; }
    bool isArmReached() const { return arm_reached_; }
    bool isFullyReached() const { return base_reached_ && arm_reached_; }
    void resetFlags();

private:
    void baseCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Node* node_;
    int robot_id_;
    std::string ns_;

    // Target tracking & Reachability flags
    std::vector<double> target_joints_deg_;
    bool base_reached_ = false;
    bool arm_reached_ = false;
    rclcpp::Time last_cmd_time_;

    // Publishers & Subscribers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr base_speed_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

#endif // KUKA_ROBOT_INTERFACE_HPP_