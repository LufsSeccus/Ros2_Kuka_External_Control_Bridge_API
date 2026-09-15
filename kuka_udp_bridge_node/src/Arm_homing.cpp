#include <chrono>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class HomeArmsNode : public rclcpp::Node {
public:
    HomeArmsNode() : Node("home_arms_node") {
        // Create publishers targeting both robot namespaces
        pub1_ = this->create_publisher<sensor_msgs::msg::JointState>("/robot1/arm_cmd_joints", 10);
        pub2_ = this->create_publisher<sensor_msgs::msg::JointState>("/robot2/arm_cmd_joints", 10);

        RCLCPP_INFO(this->get_logger(), "Homing Node started. Waiting 1 second for network discovery...");

        // Use a timer to wait 1 second before publishing, ensuring the connection is fully established
        timer_ = this->create_wall_timer(1s, [this]() {
            this->publish_home_positions();
            this->timer_->cancel(); // Stop the timer from firing again
            rclcpp::shutdown();     // Automatically kill the node after publishing
        });
    }

private:
    void publish_home_positions() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {
            "lbr_joint_1", "lbr_joint_2", "lbr_joint_3", 
            "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"
        };
        
        // The standard homing position in degrees
        std::vector<double> home_deg = {0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0};
        
        // Convert to radians (which is what your udp_bridge expects over ROS 2)
        for (double deg : home_deg) {
            msg.position.push_back(deg * (M_PI / 180.0));
        }

        pub1_->publish(msg);
        pub2_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Homing commands published successfully to both robots. Exiting.");
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub2_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HomeArmsNode>());
    return 0;
}