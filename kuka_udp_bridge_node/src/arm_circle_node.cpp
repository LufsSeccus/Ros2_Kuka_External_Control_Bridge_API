#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "kuka_robot_interface.hpp"

using namespace std::chrono_literals;

enum class CircleState {
    INIT,
    GO_TO_CENTER,
    WAIT_CENTER,
    DRAW_CIRCLE,
    WAIT_CIRCLE_STEP,
    DONE
};

class ArmCircleDemo : public rclcpp::Node {
public:
    ArmCircleDemo() : Node("arm_circle_node"), robot1_(this, 1,{0.0, 20.0, 0.0, -90.0, 0.0, 45.0, 0.0}), state_(CircleState::INIT) {
        
        // Circle Parameters
        this->declare_parameter<double>("center_x", 0.4);
        this->declare_parameter<double>("center_y", 0.0);
        this->declare_parameter<double>("center_z", 0.5);
        this->declare_parameter<double>("radius", 0.15);
        this->declare_parameter<int>("points", 36);

        cx_ = this->get_parameter("center_x").as_double();
        cy_ = this->get_parameter("center_y").as_double();
        cz_ = this->get_parameter("center_z").as_double();
        radius_ = this->get_parameter("radius").as_double();
        total_points_ = this->get_parameter("points").as_int();
        current_point_ = 0;

        init_timer_ = this->create_wall_timer(2s, [this]() {
            init_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "=== ARM CIRCLE DEMO STARTING ===");
            robot1_.setSpeed(100.0, 100.0);
            
            // Start FSM
            state_ = CircleState::GO_TO_CENTER;
            fsm_timer_ = this->create_wall_timer(50ms, std::bind(&ArmCircleDemo::advanceFSM, this));
        });
    }

private:
    void advanceFSM() {
        switch (state_) {
            case CircleState::INIT:
                break;

            case CircleState::GO_TO_CENTER: {
                RCLCPP_INFO(this->get_logger(), "Moving to Center (X: %.2f, Y: %.2f, Z: %.2f)", cx_, cy_, cz_);
                // Pitch 90 ensures the EE points directly forward/downwards along X
                robot1_.commandArmEE(cx_, cy_, cz_, 0.0, 90.0, 0.0);
                state_ = CircleState::WAIT_CENTER;
                break;
            }

            case CircleState::WAIT_CENTER: {
                if (robot1_.isEEReached()) {
                    RCLCPP_INFO(this->get_logger(), "Center reached. Commencing circular trajectory...");
                    state_ = CircleState::DRAW_CIRCLE;
                }
                break;
            }

            case CircleState::DRAW_CIRCLE: {
                if (current_point_ >= total_points_) {
                    RCLCPP_INFO(this->get_logger(), "Circle complete. Homing...");
                    robot1_.commandArmJoints({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
                    state_ = CircleState::DONE;
                    fsm_timer_->cancel();
                    rclcpp::shutdown();
                    return;
                }

                // Calculate angle for current step
                double angle_rad = (2.0 * M_PI * current_point_) / total_points_;
                
                // Circle exists in the Y-Z plane
                double target_y = cy_ + radius_ * std::cos(angle_rad);
                double target_z = cz_ + radius_ * std::sin(angle_rad);

                RCLCPP_INFO(this->get_logger(), "[Step %d/%d] Y: %.3f | Z: %.3f", current_point_ + 1, total_points_, target_y, target_z);
                
                robot1_.commandArmEE(cx_, target_y, target_z, 0.0, 90.0, 0.0);
                
                state_ = CircleState::WAIT_CIRCLE_STEP;
                current_point_++;
                break;
            }

            case CircleState::WAIT_CIRCLE_STEP: {
                if (robot1_.isEEReached()) {
                    state_ = CircleState::DRAW_CIRCLE;
                }
                break;
            }

            case CircleState::DONE:
                break;
        }
    }

    KukaRobotInterface robot1_;
    CircleState state_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;

    double cx_, cy_, cz_, radius_;
    int total_points_;
    int current_point_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmCircleDemo>());
    return 0;
}