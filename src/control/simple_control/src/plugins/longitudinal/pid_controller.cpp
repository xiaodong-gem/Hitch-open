#include <simple_control/controller_interface.hpp>
#include <boost/any.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/path.hpp"

namespace control
{
    namespace longitudinal
    {

        class PIDLonController : public ControllerInterface
        {
        public:
            PIDLonController() = default; // Add default constructor

            void setup(rclcpp::Node *node) override
            {
                ControllerInterface::setup(node);
                // Get PID parameters
                kp_ = node->declare_parameter("longitudinal.pid.kp", 0.5);
                ki_ = node->declare_parameter("longitudinal.pid.ki", 0.1);
                kd_ = node->declare_parameter("longitudinal.pid.kd", 0.1);

                // Get acceleration limits and step sizes
                max_accel_ = node->declare_parameter("longitudinal.pid.max_accel", 5.0);
                max_decel_ = node->declare_parameter("longitudinal.pid.max_decel", -5.0);
                accel_step_ = node->declare_parameter("longitudinal.pid.accel_step", 0.1);
                decel_step_ = node->declare_parameter("longitudinal.pid.decel_step", 0.1);

                max_velocity_ = node->declare_parameter("longitudinal.pid.max_velocity", 80.0);
                
                // Initialize error terms
                prev_error_ = 0.0;
                integral_error_ = 0.0;
                last_time_ = node->now();
                
                // Initialize last acceleration value
                last_accel_ = 0.0;
            }

            void runStep(const State::SharedPtr state,
                         autoware_control_msgs::msg::Control &control) override
            {
                try
                {
                    if (!state->vehicle_state)
                    {
                        // RCLCPP_DEBUG(logger_, "Unable to run PID controller because vehicle state is not available");
                        return;
                    }
                    if (!state->local_path)
                    {
                        // RCLCPP_DEBUG(logger_, "Unable to run PID controller because local path is not available");
                        return;
                    }
                    const auto vehicle_state = state->vehicle_state;
                    const auto path = state->local_path;

                    // Get target velocity from path, capped by max_velocity_
                    double target_velocity = std::min(static_cast<double>(path->points.back().longitudinal_velocity_mps), max_velocity_);

                    // Calculate time delta
                    auto current_time = parent_node_->now();
                    double dt = (current_time - last_time_).seconds();
                    last_time_ = current_time;

                    // Calculate error
                    double error = target_velocity - vehicle_state->velocity;

                    // PID calculations
                    integral_error_ += error * dt;
                    double derivative_error = (error - prev_error_) / dt;

                    // Calculate desired acceleration
                    double desired_accel = kp_ * error +
                                          ki_ * integral_error_ +
                                          kd_ * derivative_error;

                    // Apply linear acceleration change
                    if (desired_accel > last_accel_) {
                        // Accelerating - increase acceleration gradually
                        last_accel_ = std::min(last_accel_ + accel_step_, desired_accel);
                        last_accel_ = std::min(last_accel_, max_accel_);
                    } else if (desired_accel < last_accel_) {
                        // Decelerating - decrease acceleration gradually
                        last_accel_ = std::max(last_accel_ - decel_step_, desired_accel);
                        last_accel_ = std::max(last_accel_, max_decel_);
                    }
                    
                    // RCLCPP_DEBUG_STREAM(logger_, 
                    //     "Target: " << target_velocity << " km/h, "
                    //     "Current: " << vehicle_state->velocity << " km/h, "
                    //     "Accel: " << last_accel_ << " m/s²");

                    // Update control command
                    control.longitudinal.acceleration = last_accel_;
                    control.longitudinal.is_defined_acceleration = true;

                    // Store error for next iteration
                    prev_error_ = error;
                }
                catch (const boost::bad_any_cast &e)
                {
                    RCLCPP_ERROR(logger_,
                                 "Error casting state variables: %s", e.what());
                }
            }

            const char *get_plugin_name() override
            {
                return "pid_controller";
            }

        private:
            // PID gains
            double kp_;
            double ki_;
            double kd_;

            // Acceleration parameters
            double max_accel_;    // 最大加速度 (m/s²)
            double max_decel_;    // 最大减速度 (m/s², 负值)
            double accel_step_;   // 加速度变化步长 (m/s²)
            double decel_step_;   // 减速度变化步长 (m/s²)

            // Error terms
            double prev_error_;
            double integral_error_;

            // Timing
            rclcpp::Time last_time_;

            // Max velocity
            double max_velocity_;
            
            // Last acceleration value
            double last_accel_;
        };
    } // namespace longitudinal
} // namespace control

PLUGINLIB_EXPORT_CLASS(control::longitudinal::PIDLonController, control::ControllerInterface)