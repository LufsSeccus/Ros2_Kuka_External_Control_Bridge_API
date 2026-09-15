#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <random>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

enum class TurnState {
    INIT_BOTH_ROBOTS,
    START_TURN,
    WAIT_MOVE_OUT,
    WAIT_STRIKE,
    WAIT_RECOVER,
    WAIT_BASE_RETURN,
    START_DANCE,
    WAIT_DANCE_STEP,
    DONE
};

enum StrikeType {
    STRIKE_UP = 0,
    STRIKE_DOWN = 1,
    STRIKE_CW = 2,
    STRIKE_CCW = 3
};

class BadmintonDemo : public rclcpp::Node {
public:
    BadmintonDemo() : Node("badminton_demo"), demo_state_(TurnState::INIT_BOTH_ROBOTS), turn_count_(0), active_robot_(1), dance_step_count_(0) {
        
        // Declare Parameters
        this->declare_parameter<int>("max_turns", 10);
        this->declare_parameter<double>("arm_speed_pct", 100.0);  // Default 100%
        this->declare_parameter<double>("base_speed_pct", 100.0); // Default 100%

        max_turns_ = this->get_parameter("max_turns").as_int();

        state_start_time_ = this->now();

        // Publishers & Subscribers for Robot 1
        goal_pub_1_       = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot1/goal_pose", 10);
        arm_pub_1_        = this->create_publisher<sensor_msgs::msg::JointState>("/robot1/arm_cmd_joints", 10);
        arm_speed_pub_1_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/robot1/arm_speed", 10);
        base_speed_pub_1_ = this->create_publisher<std_msgs::msg::Float64>("/robot1/base_speed", 10);
        base_sub_1_       = this->create_subscription<std_msgs::msg::Bool>(
            "/robot1/base_target_reached", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { baseCallback(1, msg); });
        joint_sub_1_      = this->create_subscription<sensor_msgs::msg::JointState>(
            "/robot1/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointCallback(1, msg); });

        // Publishers & Subscribers for Robot 2
        goal_pub_2_       = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);
        arm_pub_2_        = this->create_publisher<sensor_msgs::msg::JointState>("/robot2/arm_cmd_joints", 10);
        arm_speed_pub_2_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/robot2/arm_speed", 10);
        base_speed_pub_2_ = this->create_publisher<std_msgs::msg::Float64>("/robot2/base_speed", 10);
        base_sub_2_       = this->create_subscription<std_msgs::msg::Bool>(
            "/robot2/base_target_reached", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) { baseCallback(2, msg); });
        joint_sub_2_      = this->create_subscription<sensor_msgs::msg::JointState>(
            "/robot2/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointCallback(2, msg); });

        // =========================================================================
        // [ ARM STAGE CONFIGURATIONS (DEGREES) ]
        // =========================================================================
        homing_state_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        ready_state_  = {0.0, -30.0, 0.0, -114.0, 0.0, 0.0, 55.0}; 

        strike_up_ = {
            {0.0, 30.0, 0.0, -114.0, 0.0, 55.0, 0.0}, 
            {0.0, 30.0, 0.0, -80.0, 0.0, 0.0, 0.0}, 
            ready_state_                         
        };

        strike_down_ = {
            {0.0, -55.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
            {0.0, -55.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
            ready_state_                         
        };

        strike_cw_ = {
            {-80.0, 30.0, 0.0, -80.0, 0.0, 0.0, 0.0}, 
            {0.0, 30.0, 0.0, -80.0, 0.0, 0.0, 0.0}, 
            ready_state_                         
        };

        strike_ccw_ = {
            {80.0, 30.0, 0.0, -80.0, 0.0, 0.0, 0.0}, 
            {0.0, 30.0, 0.0, -80.0, 0.0, 0.0, 0.0}, 
            ready_state_                         
        };

        all_strikes_ = {strike_up_, strike_down_, strike_cw_, strike_ccw_};
        r1_target_joints_ = homing_state_;
        r2_target_joints_ = homing_state_;

        // Random generators
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator_   = std::mt19937(seed);
        strike_dist_ = std::uniform_int_distribution<int>(0, 3);
        
        // Active Robot move limits (+/- 0.3m, +/- 15 deg)
        pos_dist_    = std::uniform_real_distribution<double>(-0.3, 0.3);
        yaw_dist_    = std::uniform_real_distribution<double>(-15.0, 15.0);

        // Idle Robot micro-shuffle limits (+/- 0.15m, +/- 5 deg)
        idle_pos_dist_ = std::uniform_real_distribution<double>(-0.15, 0.15);
        idle_yaw_dist_ = std::uniform_real_distribution<double>(-5.0, 5.0);

        // Dance Arm random jitter (+/- 15 degrees)
        arm_dance_dist_ = std::uniform_real_distribution<double>(-15.0, 15.0);

        // Initial delay to connect
        init_timer_ = this->create_wall_timer(2s, [this]() {
            init_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "=== BADMINTON DEMO STARTING ===");
            startInitialization();
        });
    }

    ~BadmintonDemo() {
        RCLCPP_WARN(this->get_logger(), "Node destroyed! Forcing both arms to HOMING state...");
        commandArm(1, homing_state_);
        commandArm(2, homing_state_);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

private:
    void publishSpeedParameters() {
        double arm_pct  = this->get_parameter("arm_speed_pct").as_double();
        double base_pct = this->get_parameter("base_speed_pct").as_double();

        double arm_ratio  = std::max(0.01, std::min(1.0, arm_pct / 100.0));
        double base_ratio = std::max(0.01, std::min(1.0, base_pct / 100.0));

        auto arm_msg = std_msgs::msg::Float64MultiArray();
        arm_msg.data = {arm_ratio, arm_ratio, arm_ratio, arm_ratio, arm_ratio, arm_ratio, arm_ratio};

        auto base_msg = std_msgs::msg::Float64();
        base_msg.data = base_ratio;

        arm_speed_pub_1_->publish(arm_msg);
        base_speed_pub_1_->publish(base_msg);
        arm_speed_pub_2_->publish(arm_msg);
        base_speed_pub_2_->publish(base_msg);

        RCLCPP_INFO(this->get_logger(), "Configured Speed Limits -> Arm: %.1f%%, Base: %.1f%%", arm_pct, base_pct);
    }

    void startInitialization() {
        demo_state_ = TurnState::INIT_BOTH_ROBOTS;
        resetFlags();
        
        publishSpeedParameters();

        commandBase(1, 0.0, 0.0, 0.0);
        commandBase(2, 0.0, 0.0, 0.0);
        
        commandArm(1, ready_state_);
        commandArm(2, ready_state_);
        
        RCLCPP_INFO(this->get_logger(), "Initializing... Waiting for bases at 0.0 and arms at Ready state.");
    }

    std::vector<double> getRandomDanceArm() {
        std::vector<double> dance_arm = ready_state_;
        for (size_t i = 0; i < 7; ++i) {
            dance_arm[i] += arm_dance_dist_(generator_);
        }
        return dance_arm;
    }

    void advanceFSM() {
        switch (demo_state_) {
            
            case TurnState::INIT_BOTH_ROBOTS: {
                if (r1_base_reached_ && r1_arm_reached_ && r2_base_reached_ && r2_arm_reached_) {
                    RCLCPP_INFO(this->get_logger(), "Init complete. Starting Robot 1 Turn.");
                    demo_state_ = TurnState::START_TURN;
                    active_robot_ = 1;
                    turn_count_ = 0;
                    advanceFSM(); 
                }
                break;
            }

            case TurnState::START_TURN: {
                if (turn_count_ >= max_turns_) {
                    RCLCPP_INFO(this->get_logger(), "=== RALLY COMPLETE! STARTING VICTORY DANCE (5.2m total, -0.4m/step) ===");
                    demo_state_ = TurnState::START_DANCE;
                    dance_step_count_ = 0;
                    r1_dance_x_ = 0.0; r1_dance_yaw_ = 0.0;
                    r2_dance_x_ = 0.0; r2_dance_yaw_ = 0.0;
                    advanceFSM();
                    return;
                }

                resetFlags();
                demo_state_ = TurnState::WAIT_MOVE_OUT;

                if (active_robot_ == 1 && turn_count_ == 0) {
                    active_strike_idx_ = STRIKE_DOWN;
                } else {
                    active_strike_idx_ = strike_dist_(generator_);
                }

                int idle_robot = (active_robot_ == 1) ? 2 : 1;

                RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Dispatching Strike %d. Idle Robot %d executing micro-shuffle.", 
                            turn_count_, active_robot_, active_strike_idx_, idle_robot);

                // 1. Dispatch Active Robot
                commandBase(active_robot_, pos_dist_(generator_), pos_dist_(generator_), yaw_dist_(generator_));
                commandArm(active_robot_, all_strikes_[active_strike_idx_][0]);

                // 2. Dispatch Idle Robot (Micro-shuffle)
                commandBase(idle_robot, idle_pos_dist_(generator_), idle_pos_dist_(generator_), idle_yaw_dist_(generator_));
                break;
            }

            case TurnState::WAIT_MOVE_OUT: {
                if (activeBaseReached() && activeArmReached()) {
                    RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Base ready. Commencing Strike Stage 2 (Hit).", turn_count_, active_robot_);
                    resetFlags();
                    demo_state_ = TurnState::WAIT_STRIKE;

                    commandArm(active_robot_, all_strikes_[active_strike_idx_][1]);

                    int idle_robot = (active_robot_ == 1) ? 2 : 1;
                    commandBase(idle_robot, 0.0, 0.0, 0.0);
                }
                break;
            }

            case TurnState::WAIT_STRIKE: {
                if (activeArmReached()) {
                    RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Strike complete. Recovering to Stage 3 (Ready).", turn_count_, active_robot_);
                    resetFlags();
                    demo_state_ = TurnState::WAIT_RECOVER;
                    commandArm(active_robot_, all_strikes_[active_strike_idx_][2]);
                }
                break;
            }

            case TurnState::WAIT_RECOVER: {
                if (activeArmReached()) {
                    RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Arm recovered. Active Base returning to 0.0.", turn_count_, active_robot_);
                    resetFlags();
                    demo_state_ = TurnState::WAIT_BASE_RETURN;
                    commandBase(active_robot_, 0.0, 0.0, 0.0);
                }
                break;
            }

            case TurnState::WAIT_BASE_RETURN: {
                if (activeBaseReached()) {
                    RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Base returned. Ending turn.", turn_count_, active_robot_);
                    
                    if (active_robot_ == 2) {
                        turn_count_++;
                    }
                    active_robot_ = (active_robot_ == 1) ? 2 : 1;
                    
                    demo_state_ = TurnState::START_TURN;
                    advanceFSM();
                }
                break;
            }

            case TurnState::START_DANCE: {
                if (dance_step_count_ >= total_dance_steps_) {
                    RCLCPP_INFO(this->get_logger(), "=== VICTORY DANCE COMPLETE! HOMING AND EXITING ===");
                    demo_state_ = TurnState::DONE;
                    commandArm(1, homing_state_);
                    commandArm(2, homing_state_);
                    rclcpp::shutdown();
                    return;
                }

                resetFlags();
                demo_state_ = TurnState::WAIT_DANCE_STEP;
                dance_step_count_++;

                // Step parameters: -0.4m X direction, -180 deg CW rotation per step
                r1_dance_x_ -= 0.4; r1_dance_yaw_ -= 180.0;
                r2_dance_x_ -= 0.4; r2_dance_yaw_ -= 180.0;

                RCLCPP_INFO(this->get_logger(), "[Dance Step %d/%d] Moving X: %0.1fm | Yaw: %0.0f deg | Random Arms", 
                            dance_step_count_, total_dance_steps_, r1_dance_x_, r1_dance_yaw_);

                // Dispatch both bases and both random arms together
                commandBase(1, r1_dance_x_, 0.0, r1_dance_yaw_);
                commandBase(2, r2_dance_x_, 0.0, r2_dance_yaw_);

                commandArm(1, getRandomDanceArm());
                commandArm(2, getRandomDanceArm());
                break;
            }

            case TurnState::WAIT_DANCE_STEP: {
                // Wait for BOTH robots (both bases and both arms) to complete current dance step
                if (r1_base_reached_ && r1_arm_reached_ && r2_base_reached_ && r2_arm_reached_) {
                    demo_state_ = TurnState::START_DANCE;
                    advanceFSM();
                }
                break;
            }

            case TurnState::DONE:
                break;
        }
    }

    void baseCallback(int id, const std_msgs::msg::Bool::SharedPtr msg) {
        if ((this->now() - state_start_time_).seconds() < 0.5) return;
        if (msg->data) {
            if (id == 1 && !r1_base_reached_) {
                r1_base_reached_ = true;
                advanceFSM();
            } else if (id == 2 && !r2_base_reached_) {
                r2_base_reached_ = true;
                advanceFSM();
            }
        }
    }

    void jointCallback(int id, const sensor_msgs::msg::JointState::SharedPtr msg) {
        if ((this->now() - state_start_time_).seconds() < 0.5) return;
        if (msg->position.size() < 7) return;

        const auto& target = (id == 1) ? r1_target_joints_ : r2_target_joints_;
        
        double max_err = 0.0;
        for (size_t i = 0; i < 7; ++i) {
            double err = std::abs((target[i] * (M_PI / 180.0)) - msg->position[i]);
            if (err > max_err) max_err = err;
        }

        if (max_err < 0.035) { 
            if (id == 1 && !r1_arm_reached_) {
                r1_arm_reached_ = true;
                advanceFSM();
            } else if (id == 2 && !r2_arm_reached_) {
                r2_arm_reached_ = true;
                advanceFSM();
            }
        }
    }

    void commandBase(int id, double x, double y, double yaw_deg) {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "odom";
        
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_deg * (M_PI / 180.0));
        pose_msg.pose.orientation.x = q.x(); 
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z(); 
        pose_msg.pose.orientation.w = q.w();

        if (id == 1) goal_pub_1_->publish(pose_msg);
        else goal_pub_2_->publish(pose_msg);
    }

    void commandArm(int id, const std::vector<double>& target_deg) {
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        joint_msg.name = {"lbr_joint_1", "lbr_joint_2", "lbr_joint_3", "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"};
        
        for (double deg : target_deg) {
            joint_msg.position.push_back(deg * (M_PI / 180.0));
        }

        if (id == 1) {
            r1_target_joints_ = target_deg;
            arm_pub_1_->publish(joint_msg);
        } else {
            r2_target_joints_ = target_deg;
            arm_pub_2_->publish(joint_msg);
        }
    }

    void resetFlags() {
        r1_base_reached_ = r2_base_reached_ = false;
        r1_arm_reached_  = r2_arm_reached_  = false;
        state_start_time_ = this->now();
    }

    bool activeBaseReached() { return active_robot_ == 1 ? r1_base_reached_ : r2_base_reached_; }
    bool activeArmReached()  { return active_robot_ == 1 ? r1_arm_reached_  : r2_arm_reached_; }

    TurnState demo_state_;
    int turn_count_;
    int active_robot_;
    int max_turns_;
    int active_strike_idx_;

    // Victory Dance Variables
    int dance_step_count_;
    const int total_dance_steps_ = 13;
    double r1_dance_x_ = 0.0, r1_dance_yaw_ = 0.0;
    double r2_dance_x_ = 0.0, r2_dance_yaw_ = 0.0;

    std::vector<double> homing_state_;
    std::vector<double> ready_state_;
    std::vector<std::vector<double>> strike_up_, strike_down_, strike_cw_, strike_ccw_;
    std::vector<std::vector<std::vector<double>>> all_strikes_;

    std::vector<double> r1_target_joints_;
    std::vector<double> r2_target_joints_;
    bool r1_base_reached_ = false, r1_arm_reached_ = false;
    bool r2_base_reached_ = false, r2_arm_reached_ = false;
    rclcpp::Time state_start_time_;

    std::mt19937 generator_;
    std::uniform_int_distribution<int> strike_dist_;
    std::uniform_real_distribution<double> pos_dist_;
    std::uniform_real_distribution<double> yaw_dist_;
    std::uniform_real_distribution<double> idle_pos_dist_;
    std::uniform_real_distribution<double> idle_yaw_dist_;
    std::uniform_real_distribution<double> arm_dance_dist_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_1_, goal_pub_2_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_pub_1_, arm_pub_2_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_speed_pub_1_, arm_speed_pub_2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr base_speed_pub_1_, base_speed_pub_2_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_sub_1_, base_sub_2_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_1_, joint_sub_2_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BadmintonDemo>());
    return 0;
}