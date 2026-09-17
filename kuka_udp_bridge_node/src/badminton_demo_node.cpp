#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <random>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "kuka_robot_interface.hpp"

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
    START_RETURN,
    WAIT_RETURN_STEP,
    FINAL_BASE_ALIGN,
    WAIT_FINAL_BASE_ALIGN,
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
    BadmintonDemo() 
        : Node("badminton_demo"),
          homing_state_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
          ready_state_({0.0, -30.0, 0.0, -114.0, 0.0, 0.0, 55.0}),
          robot1_(this, 1, homing_state_),
          robot2_(this, 2, homing_state_),
          demo_state_(TurnState::INIT_BOTH_ROBOTS),
          turn_count_(0),
          active_robot_(1),
          dance_step_count_(0) {
        
        // Declare Parameters
        this->declare_parameter<int>("max_turns", 10);
        this->declare_parameter<double>("arm_speed_pct", 100.0);
        this->declare_parameter<double>("base_speed_pct", 100.0);

        max_turns_ = this->get_parameter("max_turns").as_int();

        // Arm Strike Configurations
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

        // Random Generators
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator_   = std::mt19937(seed);
        strike_dist_ = std::uniform_int_distribution<int>(0, 3);
        
        random_point_x  = std::uniform_real_distribution<double>(-4.0, 4.0);
        random_point_y  = std::uniform_real_distribution<double>(0.0, 2.0);
        yaw_dist_       = std::uniform_real_distribution<double>(-15.0, 15.0);
        idle_pos_dist_  = std::uniform_real_distribution<double>(-0.15, 0.15);
        idle_yaw_dist_  = std::uniform_real_distribution<double>(-5.0, 5.0);

        // Dance arm jitter range (-30 to +30 degrees)
        arm_dance_dist_ = std::uniform_real_distribution<double>(-30.0, 30.0);

        // Discovery Delay
        init_timer_ = this->create_wall_timer(2s, [this]() {
            init_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "=== BADMINTON DEMO STARTING ===");
            startInitialization();
        });
    }

    ~BadmintonDemo() {
        RCLCPP_WARN(this->get_logger(), "Node destroyed! Forcing both arms to HOMING state...");
        robot1_.commandArmJoints(homing_state_);
        robot2_.commandArmJoints(homing_state_);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

private:
    KukaRobotInterface& getActiveRobot() { return (active_robot_ == 1) ? robot1_ : robot2_; }
    KukaRobotInterface& getIdleRobot()   { return (active_robot_ == 1) ? robot2_ : robot1_; }

    void startInitialization() {
        demo_state_ = TurnState::INIT_BOTH_ROBOTS;

        double arm_pct  = this->get_parameter("arm_speed_pct").as_double();
        double base_pct = this->get_parameter("base_speed_pct").as_double();
        robot1_.setSpeed(arm_pct, base_pct);
        robot2_.setSpeed(arm_pct, base_pct);

        robot1_.commandBase(0.0, 0.0, 0.0);
        robot2_.commandBase(0.0, 0.0, 0.0);
        
        robot1_.commandArmJoints(ready_state_);
        robot2_.commandArmJoints(ready_state_);
        
        RCLCPP_INFO(this->get_logger(), "Initializing... Waiting for bases at 0.0 and arms at Ready state.");

        // Periodic FSM tick (20 Hz)
        fsm_timer_ = this->create_wall_timer(50ms, std::bind(&BadmintonDemo::advanceFSM, this));
    }

    // Sequentially unlocks joints 1 to N based on current step_count
    std::vector<double> getStepDanceArm(int step_count) {
        std::vector<double> dance_arm = homing_state_;
        int active_joints = std::min(step_count, 7);

        for (int i = 0; i < active_joints; ++i) {
            dance_arm[i] = homing_state_[i] + arm_dance_dist_(generator_);
        }
        return dance_arm;
    }

    void advanceFSM() {
        switch (demo_state_) {
            
            case TurnState::INIT_BOTH_ROBOTS: {
                if (robot1_.isFullyReached() && robot2_.isFullyReached()) {
                    RCLCPP_INFO(this->get_logger(), "Init complete. Starting Robot 1 Turn.");
                    demo_state_ = TurnState::START_TURN;
                    active_robot_ = 1;
                    turn_count_ = 0;
                }
                break;
            }

            case TurnState::START_TURN: {
                if (turn_count_ >= max_turns_) {
                    RCLCPP_INFO(this->get_logger(), "=== RALLY COMPLETE! STARTING DANCE ===");
                    demo_state_ = TurnState::START_DANCE;
                    dance_step_count_ = 0;
                    r1_dance_x_ = 0.0; r1_dance_yaw_ = 0.0;
                    r2_dance_x_ = 0.0; r2_dance_yaw_ = 0.0;
                    break;
                }

                demo_state_ = TurnState::WAIT_MOVE_OUT;

                if (active_robot_ == 1 && turn_count_ == 0) {
                    active_strike_idx_ = STRIKE_DOWN;
                } else {
                    active_strike_idx_ = strike_dist_(generator_);
                }

                int idle_id = (active_robot_ == 1) ? 2 : 1;
                RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Dispatching Strike %d. Idle Robot %d shuffling.", 
                            turn_count_, active_robot_, active_strike_idx_, idle_id);

                // Sample and interpolate target active base move
                active_base_yaw_ = yaw_dist_(generator_);
                active_base_x_   = random_point_x(generator_);
                active_base_y_   = random_point_y(generator_);
                double distance_goal = std::hypot(active_base_x_, active_base_y_);
            
                int intermediate_points = std::max(1, static_cast<int>(std::ceil(distance_goal / 1.5)));
                double step_x   = active_base_x_ / intermediate_points;
                double step_y   = active_base_y_ / intermediate_points;    
                double step_yaw = active_base_yaw_ / intermediate_points;   

                for (int i = 1; i <= intermediate_points; i++) {
                    if (i == intermediate_points) {
                        getActiveRobot().commandBase(active_base_x_, active_base_y_, active_base_yaw_);
                    } else {
                        getActiveRobot().commandBase(i * step_x, i * step_y, i * step_yaw);
                    }
                }
                
                getActiveRobot().commandArmJoints(all_strikes_[active_strike_idx_][0]);
                getIdleRobot().commandBase(idle_pos_dist_(generator_), idle_pos_dist_(generator_), idle_yaw_dist_(generator_));
                break;
            }

            case TurnState::WAIT_MOVE_OUT: {
                if (getActiveRobot().isFullyReached()) {
                    RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Base ready. Commencing Strike Stage 2 (Hit).", turn_count_, active_robot_);
                    demo_state_ = TurnState::WAIT_STRIKE;

                    getActiveRobot().commandArmJoints(all_strikes_[active_strike_idx_][1]);
                    getIdleRobot().commandBase(0.0, 0.0, 0.0);
                }
                break;
            }

            case TurnState::WAIT_STRIKE: {
                if (getActiveRobot().isJointsArmReached()) {
                    RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Strike complete. Recovering to Stage 3 (Ready).", turn_count_, active_robot_);
                    demo_state_ = TurnState::WAIT_RECOVER;

                    getActiveRobot().commandArmJoints(all_strikes_[active_strike_idx_][2]);
                }
                break;
            }

            case TurnState::WAIT_RECOVER: {
                if (getActiveRobot().isJointsArmReached()) {
                    RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Arm recovered. Base returning to 0.0 with interpolation.", turn_count_, active_robot_);
                    demo_state_ = TurnState::WAIT_BASE_RETURN;

                    double return_dist = std::hypot(active_base_x_, active_base_y_);
                    int return_steps = std::max(1, static_cast<int>(std::ceil(return_dist / 1.5)));
                    double ret_step_x   = active_base_x_ / return_steps;
                    double ret_step_y   = active_base_y_ / return_steps;
                    double ret_step_yaw = active_base_yaw_ / return_steps;

                    for (int i = return_steps - 1; i >= 0; i--) {
                        if (i == 0) {
                            getActiveRobot().commandBase(0.0, 0.0, 0.0);
                        } else {
                            getActiveRobot().commandBase(i * ret_step_x, i * ret_step_y, i * ret_step_yaw);
                        }
                    }
                }
                break;
            }

            case TurnState::WAIT_BASE_RETURN: {
                if (getActiveRobot().isBaseReached()) {
                    RCLCPP_INFO(this->get_logger(), "[Turn %d/Robot %d] Base returned to origin. Ending turn.", turn_count_, active_robot_);
                    
                    if (active_robot_ == 2) {
                        turn_count_++;
                    }
                    active_robot_ = (active_robot_ == 1) ? 2 : 1;
                    demo_state_ = TurnState::START_TURN;
                }
                break;
            }

            case TurnState::START_DANCE: {
                if (r1_dance_x_ >= 2.6 && r2_dance_x_ <= -2.6) {
                    RCLCPP_INFO(this->get_logger(), "=== OUTWARD DANCE COMPLETE. STARTING RETURN PHASE ===");
                    demo_state_ = TurnState::START_RETURN;
                    break;
                }

                demo_state_ = TurnState::WAIT_DANCE_STEP;
                dance_step_count_++;

                r1_dance_x_ += 0.4; if (r1_dance_x_ > 2.6) r1_dance_x_ = 2.6;
                r1_dance_yaw_ -= 180.0;

                r2_dance_x_ -= 0.4; if (r2_dance_x_ < -2.6) r2_dance_x_ = -2.6;
                r2_dance_yaw_ -= 180.0;

                RCLCPP_INFO(this->get_logger(), "[Dance Step Out %d] Unlocking %d joint(s) | R1 X: %0.2fm | R2 X: %0.2fm", 
                            dance_step_count_, std::min(dance_step_count_, 7), r1_dance_x_, r2_dance_x_);

                robot1_.commandBase(r1_dance_x_, 0.0, r1_dance_yaw_);
                robot2_.commandBase(r2_dance_x_, 0.0, r2_dance_yaw_);

                robot1_.commandArmJoints(getStepDanceArm(dance_step_count_));
                robot2_.commandArmJoints(getStepDanceArm(dance_step_count_));
                break;
            }

            case TurnState::WAIT_DANCE_STEP: {
                if (robot1_.isFullyReached() && robot2_.isFullyReached()) {
                    demo_state_ = TurnState::START_DANCE;
                }
                break;
            }

            case TurnState::START_RETURN: {
                if (r1_dance_x_ <= 0.0 && r2_dance_x_ >= 0.0) {
                    RCLCPP_INFO(this->get_logger(), "=== RETURN POSITION COMPLETE. TURNING BASES BACK TO 0 DEG ORIGIN ===");
                    demo_state_ = TurnState::FINAL_BASE_ALIGN;
                    break;
                }

                demo_state_ = TurnState::WAIT_RETURN_STEP;
                dance_step_count_++;

                r1_dance_x_ -= 0.4; if (r1_dance_x_ < 0.0) r1_dance_x_ = 0.0;
                r2_dance_x_ += 0.4; if (r2_dance_x_ > 0.0) r2_dance_x_ = 0.0;

                RCLCPP_INFO(this->get_logger(), "[Return Step %d] R1 X: %0.2fm | R2 X: %0.2fm", dance_step_count_, r1_dance_x_, r2_dance_x_);

                robot1_.commandBase(r1_dance_x_, 0.0, r1_dance_yaw_);
                robot2_.commandBase(r2_dance_x_, 0.0, r2_dance_yaw_);

                robot1_.commandArmJoints(getStepDanceArm(dance_step_count_));
                robot2_.commandArmJoints(getStepDanceArm(dance_step_count_));
                break;
            }

            case TurnState::WAIT_RETURN_STEP: {
                if (robot1_.isFullyReached() && robot2_.isFullyReached()) {
                    demo_state_ = TurnState::START_RETURN;
                }
                break;
            }

            case TurnState::FINAL_BASE_ALIGN: {
                demo_state_ = TurnState::WAIT_FINAL_BASE_ALIGN;

                // Reset base rotation back to 0.0 degrees at origin (0.0, 0.0)
                robot1_.commandBase(0.0, 0.0, 0.0);
                robot2_.commandBase(0.0, 0.0, 0.0);

                // Command arms to clean homing position
                robot1_.commandArmJoints(homing_state_);
                robot2_.commandArmJoints(homing_state_);
                break;
            }

            case TurnState::WAIT_FINAL_BASE_ALIGN: {
                if (robot1_.isFullyReached() && robot2_.isFullyReached()) {
                    RCLCPP_INFO(this->get_logger(), "=== DEMO COMPLETE. BASES HOMED AT 0.0 ORIGIN. EXITING ===");
                    demo_state_ = TurnState::DONE;
                    fsm_timer_->cancel();
                    rclcpp::shutdown();
                }
                break;
            }

            case TurnState::DONE:
                break;
        }
    }

    // Default Pose Vectors
    const std::vector<double> homing_state_;
    const std::vector<double> ready_state_;

    // Robot Interface Handles
    KukaRobotInterface robot1_;
    KukaRobotInterface robot2_;

    // FSM State Variables
    TurnState demo_state_;
    int turn_count_;
    int active_robot_;
    int max_turns_;
    int active_strike_idx_;

    // Target Tracking
    double active_base_x_   = 0.0;
    double active_base_y_   = 0.0;
    double active_base_yaw_ = 0.0;

    // Dance Variables
    int dance_step_count_;
    double r1_dance_x_ = 0.0, r1_dance_yaw_ = 0.0;
    double r2_dance_x_ = 0.0, r2_dance_yaw_ = 0.0;

    std::vector<std::vector<double>> strike_up_, strike_down_, strike_cw_, strike_ccw_;
    std::vector<std::vector<std::vector<double>>> all_strikes_;

    // Random Generators
    std::mt19937 generator_;
    std::uniform_int_distribution<int> strike_dist_;
    std::uniform_real_distribution<double> random_point_x;
    std::uniform_real_distribution<double> random_point_y;
    std::uniform_real_distribution<double> yaw_dist_;
    std::uniform_real_distribution<double> idle_pos_dist_;
    std::uniform_real_distribution<double> idle_yaw_dist_;
    std::uniform_real_distribution<double> arm_dance_dist_;

    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BadmintonDemo>());
    return 0;
}