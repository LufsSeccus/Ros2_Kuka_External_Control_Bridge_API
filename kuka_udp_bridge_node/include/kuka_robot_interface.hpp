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

enum class ArmControlMode {
    JOINTS,
    CARTESIAN
};

class KukaRobotInterface {
public:
    KukaRobotInterface(rclcpp::Node* node_ptr, int robot_id, const std::vector<double>& default_joints);

    // Command API
    void commandBase(double x, double y, double yaw_deg);
    void commandArmJoints(const std::vector<double>& target_deg);
    void commandArmEE(double x, double y, double z, double roll_deg, double pitch_deg, double yaw_deg);
    void setSpeed(double arm_pct, double base_pct);

    // Explicit State Tracking API
    bool isBaseReached() const { return base_reached_; }
    bool isJointsArmReached() const { return joints_reached_; }
    bool isEEReached() const { return ee_reached_; }
    bool isFullyReached() const;
    void resetFlags();

private:
    void baseCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void armPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Node* node_;
    int robot_id_;
    std::string ns_;

    // Target tracking & Reachability flags
    ArmControlMode arm_mode_ = ArmControlMode::JOINTS;
    std::vector<double> target_joints_deg_;
    double target_ee_x_ = 0.0, target_ee_y_ = 0.0, target_ee_z_ = 0.0;
    
    bool base_reached_ = false;
    bool joints_reached_ = false;
    bool ee_reached_ = false;
    rclcpp::Time last_cmd_time_;

    // Publishers & Subscribers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr base_speed_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_sub_;
};

#endif // KUKA_ROBOT_INTERFACE_HPP_