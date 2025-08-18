#include <simple_control/controller_interface.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <boost/any.hpp>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

namespace control
{
    namespace lateral
    {
        class MPCController : public ControllerInterface
        {
        public:
            MPCController() = default;

            void setup(rclcpp::Node *node) override
            {
                ControllerInterface::setup(node);
                
                // MPC parameters
                prediction_horizon_ = node->declare_parameter("lateral.mpc.prediction_horizon", 10);
                control_horizon_ = node->declare_parameter("lateral.mpc.control_horizon", 3);
                dt_ = node->declare_parameter("lateral.mpc.dt", 0.1);
                wheel_base_ = node->declare_parameter("lateral.mpc.wheel_base", 2.7);
                max_steering_angle_ = node->declare_parameter("lateral.mpc.max_steering_angle", 0.52);
                
                // Cost function weights
                q_cte_ = node->declare_parameter("lateral.mpc.q_cte", 1.0);
                q_epsi_ = node->declare_parameter("lateral.mpc.q_epsi", 1.0);
                r_delta_ = node->declare_parameter("lateral.mpc.r_delta", 0.1);
                r_ddelta_ = node->declare_parameter("lateral.mpc.r_ddelta", 0.1);
                
                previous_steering_ = 0.0;
            }

            void runStep(const State::SharedPtr state,
                         autoware_control_msgs::msg::Control &control) override
            {
                try
                {
                    if (!state->vehicle_state || !state->local_path)
                    {
                        return;
                    }
                    
                    const auto vehicle_state = state->vehicle_state;
                    const auto path = state->local_path;
                    
                    if (path->points.empty())
                    {
                        return;
                    }

                    // Find closest point and calculate errors
                    auto closest_idx = findClosestPoint(vehicle_state, path);
                    auto errors = calculateErrors(vehicle_state, path, closest_idx);
                    
                    // Get reference trajectory
                    auto ref_trajectory = getReferencePath(path, closest_idx, prediction_horizon_);
                    
                    // Solve MPC
                    double steering = solveMPC(vehicle_state, errors, ref_trajectory);
                    
                    // Apply constraints
                    steering = std::clamp(steering, -max_steering_angle_, max_steering_angle_);
                    
                    RCLCPP_DEBUG_STREAM(logger_, "MPC steering: " << (steering * 180.0 / M_PI) << " deg");
                    
                    control.lateral.steering_tire_angle = steering;
                    control.lateral.is_defined_steering_tire_rotation_rate = false;
                    
                    previous_steering_ = steering;
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(logger_, "MPC Error: %s", e.what());
                }
            }

            const char *get_plugin_name() override
            {
                return "mpc_controller";
            }

        private:
            // MPC parameters
            int prediction_horizon_;
            int control_horizon_;
            double dt_;
            double wheel_base_;
            double max_steering_angle_;
            
            // Cost weights
            double q_cte_;
            double q_epsi_;
            double r_delta_;
            double r_ddelta_;
            
            double previous_steering_;

            struct MPCErrors
            {
                double cte;      // Cross track error
                double epsi;     // Heading error
                double velocity; // Current velocity
            };

            struct ReferencePoint
            {
                double x, y, psi;
                double curvature;
            };

            size_t findClosestPoint(const VehicleState::SharedPtr vehicle,
                                   const autoware_planning_msgs::msg::Path::SharedPtr path)
            {
                double min_distance = std::numeric_limits<double>::max();
                size_t closest_idx = 0;
                
                for (size_t i = 0; i < path->points.size(); ++i)
                {
                    double dx = path->points[i].pose.position.x - vehicle->x;
                    double dy = path->points[i].pose.position.y - vehicle->y;
                    double distance = std::sqrt(dx * dx + dy * dy);
                    
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        closest_idx = i;
                    }
                }
                
                return closest_idx;
            }

            MPCErrors calculateErrors(const VehicleState::SharedPtr vehicle,
                                     const autoware_planning_msgs::msg::Path::SharedPtr path,
                                     size_t closest_idx)
            {
                MPCErrors errors;
                
                if (closest_idx >= path->points.size())
                {
                    errors.cte = 0.0;
                    errors.epsi = 0.0;
                    errors.velocity = vehicle->velocity;
                    return errors;
                }

                // Get reference point
                const auto& ref_point = path->points[closest_idx];
                double ref_x = ref_point.pose.position.x;
                double ref_y = ref_point.pose.position.y;
                
                // Calculate reference heading
                double ref_psi = 0.0;
                if (closest_idx + 1 < path->points.size())
                {
                    const auto& next_point = path->points[closest_idx + 1];
                    ref_psi = atan2(next_point.pose.position.y - ref_y,
                                   next_point.pose.position.x - ref_x);
                }

                // Transform to vehicle coordinate system
                double dx = vehicle->x - ref_x;
                double dy = vehicle->y - ref_y;
                
                // Cross track error (distance to reference line)
                errors.cte = dy * cos(ref_psi) - dx * sin(ref_psi);
                
                // Heading error
                double psi_error = vehicle->yaw - ref_psi;
                errors.epsi = atan2(sin(psi_error), cos(psi_error)); // Normalize
                
                errors.velocity = vehicle->velocity;
                
                return errors;
            }

            std::vector<ReferencePoint> getReferencePath(const autoware_planning_msgs::msg::Path::SharedPtr path,
                                                        size_t start_idx, int horizon)
            {
                std::vector<ReferencePoint> ref_path;
                
                for (int i = 0; i < horizon; ++i)
                {
                    size_t idx = std::min(start_idx + i, path->points.size() - 1);
                    
                    ReferencePoint ref_point;
                    ref_point.x = path->points[idx].pose.position.x;
                    ref_point.y = path->points[idx].pose.position.y;
                    
                    // Calculate reference heading
                    if (idx + 1 < path->points.size())
                    {
                        ref_point.psi = atan2(path->points[idx + 1].pose.position.y - ref_point.y,
                                             path->points[idx + 1].pose.position.x - ref_point.x);
                    }
                    else if (idx > 0)
                    {
                        ref_point.psi = atan2(ref_point.y - path->points[idx - 1].pose.position.y,
                                             ref_point.x - path->points[idx - 1].pose.position.x);
                    }
                    else
                    {
                        ref_point.psi = 0.0;
                    }
                    
                    ref_point.curvature = 0.0; // Simplified
                    
                    ref_path.push_back(ref_point);
                }
                
                return ref_path;
            }

            double solveMPC(const VehicleState::SharedPtr vehicle,
                           const MPCErrors& errors,
                           const std::vector<ReferencePoint>& ref_trajectory)
            {
                // Simplified MPC using kinematic bicycle model
                // State: [x, y, psi, v, cte, epsi]
                // Control: [delta]
                
                const int nx = 6; // state dimension
                const int nu = 1; // control dimension
                const int N = prediction_horizon_;
                
                // For simplicity, use analytical solution for basic MPC
                // In practice, you would use quadratic programming solver
                
                double v = std::max(vehicle->velocity, 0.1); // Avoid division by zero
                
                // Simple feedback control with prediction
                double steering_ff = 0.0; // Feedforward term
                
                // Proportional control for cross track error
                double k_cte = q_cte_ / (q_cte_ + r_delta_);
                double delta_cte = -k_cte * errors.cte;
                
                // Proportional control for heading error  
                double k_epsi = q_epsi_ / (q_epsi_ + r_delta_);
                double delta_epsi = -k_epsi * errors.epsi;
                
                // Predictive term (simplified)
                double prediction_time = dt_ * control_horizon_;
                double predicted_cte = errors.cte + v * sin(errors.epsi) * prediction_time;
                double delta_pred = -0.1 * predicted_cte;
                
                // Combine terms
                double total_steering = delta_cte + delta_epsi + delta_pred + steering_ff;
                
                // Smooth the control input
                double alpha = 0.3; // Smoothing factor
                total_steering = alpha * total_steering + (1 - alpha) * previous_steering_;
                
                return total_steering;
            }
        };

    } // namespace lateral
} // namespace control

PLUGINLIB_EXPORT_CLASS(control::lateral::MPCController, control::ControllerInterface)