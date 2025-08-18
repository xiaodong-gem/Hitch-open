#include "autoware_lgsvl_interface/autoware_lgsvl_interface.hpp"

// 新增：CanBusData 与 VehicleControlData
#include "lgsvl_msgs/msg/can_bus_data.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

namespace autoware_lgsvl_interface
{

AutowareLgsvlInterface::AutowareLgsvlInterface(
  const rclcpp::NodeOptions & options)
: Node("autoware_lgsvl_interface", options)
{
  this->declare_parameter("max_accel_mps2", 3.0f);
  this->declare_parameter("max_decel_mps2", -3.0f);

  max_accel = this->get_parameter("max_accel_mps2").as_double();
  max_decel = this->get_parameter("max_decel_mps2").as_double();

  // Publishers
  m_lgsvl_control_pub = create_publisher<lgsvl_msgs::msg::VehicleControlData>(
    "vehicle_control_cmd", rclcpp::QoS{10});

  // Subscribers
  m_autoware_control_sub = create_subscription<autoware_control_msgs::msg::Control>(
    "control_cmd", rclcpp::QoS{10},
    std::bind(&AutowareLgsvlInterface::on_control_msg, this, std::placeholders::_1));

  // 新增：订阅 /can 获取 engine_rpm
  m_canbus_sub = create_subscription<lgsvl_msgs::msg::CanBusData>(
    "/can", rclcpp::QoS{10},
    std::bind(&AutowareLgsvlInterface::on_can_msg, this, std::placeholders::_1));

  // 新增：发布控制档位（可选，这里直接复用 m_lgsvl_control_pub）
}

void AutowareLgsvlInterface::on_can_msg(
  const lgsvl_msgs::msg::CanBusData::SharedPtr msg)
{
  m_engine_rpm = msg->engine_rpm;

  // 升档逻辑：1~6 档
  if (m_engine_rpm > 3000.0) {
    if (m_current_gear < 6) {
      ++m_current_gear;
      RCLCPP_INFO(this->get_logger(),
                  "Shifted UP to gear %d (rpm %.0f)",
                  m_current_gear, m_engine_rpm);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Already on max gear %d", m_current_gear);
    }
  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
        "RPM too low to shift up (current: %.0f, required: >3000)", m_engine_rpm);
  }
}

void AutowareLgsvlInterface::on_control_msg(
  const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  auto lgsvl_msg = lgsvl_msgs::msg::VehicleControlData();

  lgsvl_msg.header.stamp = msg->stamp;

  // Longitudinal
  if (msg->longitudinal.acceleration >= 0.0f) {
    lgsvl_msg.acceleration_pct = std::min(msg->longitudinal.acceleration / max_accel, 1.0);
    lgsvl_msg.braking_pct = 0.0f;
  } else {
    lgsvl_msg.acceleration_pct = 0.0f;
    lgsvl_msg.braking_pct = std::abs(std::min(std::abs(msg->longitudinal.acceleration) / max_decel, 1.0));
  }

  // Lateral
  lgsvl_msg.target_wheel_angle = msg->lateral.steering_tire_angle;

  // Gear：由 Autoware 方向决定，但最终注入我们自动计算的档位
  if (std::abs(msg->longitudinal.acceleration) < 0.01f || max_accel == 0.0f) {
    lgsvl_msg.target_gear = lgsvl_msg.GEAR_NEUTRAL;
  } else if (msg->longitudinal.acceleration > 0.0f) {
    // 注入自动升档后的档位
    lgsvl_msg.target_gear = static_cast<int8_t>(m_current_gear);
  } else {
    lgsvl_msg.target_gear = lgsvl_msg.GEAR_NEUTRAL;  // brake
  }

  m_lgsvl_control_pub->publish(lgsvl_msg);
}

}  // namespace autoware_lgsvl_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<autoware_lgsvl_interface::AutowareLgsvlInterface>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}