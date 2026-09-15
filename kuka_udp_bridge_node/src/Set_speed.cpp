#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class SetSpeedNode : public rclcpp::Node {
public:
    SetSpeedNode() : Node("set_speed_node") {
        // Declare the percentage parameter with a default of 100.0
        this->declare_parameter<double>("speed_percentage", 100.0);

        // Publishers for Robot 1
        arm_pub1_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/robot1/arm_speed", 10);
        base_pub1_ = this->create_publisher<std_msgs::msg::Float64>("/robot1/base_speed", 10);
        
        // Publishers for Robot 2
        arm_pub2_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/robot2/arm_speed", 10);
        base_pub2_ = this->create_publisher<std_msgs::msg::Float64>("/robot2/base_speed", 10);

        RCLCPP_INFO(this->get_logger(), "Speed Setter Node started. Waiting 1 second for network discovery...");

        // Wait 1 second before publishing to ensure subscribers are connected
        timer_ = this->create_wall_timer(1s, [this]() {
            this->publish_speeds();
            this->timer_->cancel();
            rclcpp::shutdown();
        });
    }

private:
    void publish_speeds() {
        double speed_pct = this->get_parameter("speed_percentage").as_double();
        double ratio = speed_pct / 100.0;
        
        // Clamp the ratio between 1% (0.01) and 100% (1.0) for safety
        if (ratio < 0.01) ratio = 0.01;
        if (ratio > 1.0) ratio = 1.0;

        RCLCPP_INFO(this->get_logger(), "Setting speeds to %.1f%% (ratio: %.2f)", speed_pct, ratio);

        // Prepare Arm Message (7 joints)
        auto arm_msg = std_msgs::msg::Float64MultiArray();
        arm_msg.data = {ratio, ratio, ratio, ratio, ratio, ratio, ratio};

        // Prepare Base Message (scalar)
        auto base_msg = std_msgs::msg::Float64();
        base_msg.data = ratio;

        // Publish to Robot 1
        arm_pub1_->publish(arm_msg);
        base_pub1_->publish(base_msg);

        // Publish to Robot 2
        arm_pub2_->publish(arm_msg);
        base_pub2_->publish(base_msg);

        RCLCPP_INFO(this->get_logger(), "Speed commands published successfully. Exiting.");
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr base_pub1_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr base_pub2_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetSpeedNode>());
    return 0;
}