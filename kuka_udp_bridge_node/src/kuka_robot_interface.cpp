#include "kuka_robot_interface.hpp"

KukaRobotInterface::KukaRobotInterface(rclcpp::Node* node_ptr, int robot_id, const std::vector<double>& default_joints)
    : node_(node_ptr), robot_id_(robot_id), target_joints_deg_(default_joints) {
    
    ns_ = "/robot" + std::to_string(robot_id_);
    last_cmd_time_ = node_->now();

    // Setup Publishers
    goal_pub_       = node_->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + "/goal_pose", 10);
    arm_pose_pub_   = node_->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + "/arm_goal_pose", 10);
    
    arm_pub_        = node_->create_publisher<sensor_msgs::msg::JointState>(ns_ + "/arm_cmd_joints", 10);
    arm_speed_pub_  = node_->create_publisher<std_msgs::msg::Float64MultiArray>(ns_ + "/arm_speed", 10);
    base_speed_pub_ = node_->create_publisher<std_msgs::msg::Float64>(ns_ + "/base_speed", 10);

    // Setup Subscribers
    base_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        ns_ + "/base_target_reached", 10, std::bind(&KukaRobotInterface::baseCallback, this, std::placeholders::_1));

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        ns_ + "/joint_states", 10, std::bind(&KukaRobotInterface::jointCallback, this, std::placeholders::_1));

    // FIX 2: Updated topic name to match bridge publisher 'ee_pose_state'
    arm_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        ns_ + "/ee_pose_state", 10, std::bind(&KukaRobotInterface::armPoseCallback, this, std::placeholders::_1));
}

void KukaRobotInterface::commandBase(double x, double y, double yaw_deg) {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "odom";
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_deg * (M_PI / 180.0));
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    base_reached_ = false;
    last_cmd_time_ = node_->now();
    goal_pub_->publish(pose_msg);
}

void KukaRobotInterface::commandArmJoints(const std::vector<double>& target_deg) {
    arm_mode_ = ArmControlMode::JOINTS;
    target_joints_deg_ = target_deg;
    
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = node_->now();
    joint_msg.name = {"lbr_joint_1", "lbr_joint_2", "lbr_joint_3", "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"};

    for (double deg : target_deg) {
        joint_msg.position.push_back(deg * (M_PI / 180.0));
    }

    joints_reached_ = false;
    last_cmd_time_ = node_->now();
    arm_pub_->publish(joint_msg);
}

void KukaRobotInterface::commandArmEE(double x, double y, double z, double roll_deg, double pitch_deg, double yaw_deg) {
    arm_mode_ = ArmControlMode::CARTESIAN;
    target_ee_x_ = x;
    target_ee_y_ = y;
    target_ee_z_ = z;

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "base_link"; 
    
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll_deg * (M_PI / 180.0), pitch_deg * (M_PI / 180.0), yaw_deg * (M_PI / 180.0));
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ee_reached_ = false;
    last_cmd_time_ = node_->now();
    arm_pose_pub_->publish(pose_msg);
}

void KukaRobotInterface::setSpeed(double arm_pct, double base_pct) {
    double arm_ratio  = std::max(0.01, std::min(1.0, arm_pct / 100.0));
    double base_ratio = std::max(0.01, std::min(1.0, base_pct / 100.0));

    auto arm_msg = std_msgs::msg::Float64MultiArray();
    arm_msg.data = {arm_ratio, arm_ratio, arm_ratio, arm_ratio, arm_ratio, arm_ratio, arm_ratio};

    auto base_msg = std_msgs::msg::Float64();
    base_msg.data = base_ratio;

    arm_speed_pub_->publish(arm_msg);
    base_speed_pub_->publish(base_msg);
}

bool KukaRobotInterface::isFullyReached() const {
    if (arm_mode_ == ArmControlMode::JOINTS) {
        return base_reached_ && joints_reached_;
    }
    return base_reached_ && ee_reached_;
}

void KukaRobotInterface::resetFlags() {
    base_reached_   = false;
    joints_reached_ = false;
    ee_reached_     = false;
    last_cmd_time_  = node_->now();
}

void KukaRobotInterface::baseCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if ((node_->now() - last_cmd_time_).seconds() < 0.5) return;
    if (msg->data) {
        base_reached_ = true;
    }
}

void KukaRobotInterface::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if ((node_->now() - last_cmd_time_).seconds() < 0.5) return;
    if (msg->position.size() < 7) return;

    double max_err = 0.0;
    for (size_t i = 0; i < 7; ++i) {
        double err = std::abs((target_joints_deg_[i] * (M_PI / 180.0)) - msg->position[i]);
        if (err > max_err) max_err = err;
    }

    if (max_err < 0.035) {
        joints_reached_ = true;
    }
}

void KukaRobotInterface::armPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if ((node_->now() - last_cmd_time_).seconds() < 0.5) return;

    // Check Cartesian Euclidean Distance Error
    double dx = target_ee_x_ - msg->pose.position.x;
    double dy = target_ee_y_ - msg->pose.position.y;
    double dz = target_ee_z_ - msg->pose.position.z;
    double error = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (error < 0.015) { // 1.5 cm tolerance
        ee_reached_ = true;
    }
}