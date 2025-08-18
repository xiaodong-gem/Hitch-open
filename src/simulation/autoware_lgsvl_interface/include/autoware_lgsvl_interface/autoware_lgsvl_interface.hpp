#ifndef AUTOWARE_LGSVL_INTERFACE__AUTOWARE_LGSVL_INTERFACE_HPP_
#define AUTOWARE_LGSVL_INTERFACE__AUTOWARE_LGSVL_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <lgsvl_msgs/msg/vehicle_control_data.hpp>
#include <lgsvl_msgs/msg/can_bus_data.hpp>
#include <autoware_control_msgs/msg/control.hpp>

namespace autoware_lgsvl_interface
{

class AutowareLgsvlInterface : public rclcpp::Node
{
public:
  explicit AutowareLgsvlInterface(const rclcpp::NodeOptions & options);

private:
  /* 回调函数 */
  void on_control_msg(const autoware_control_msgs::msg::Control::SharedPtr msg);
  void on_can_msg(const lgsvl_msgs::msg::CanBusData::SharedPtr msg);

  /* 发布者 */
  rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr m_lgsvl_control_pub;

  /* 订阅者 */
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr m_autoware_control_sub;
  rclcpp::Subscription<lgsvl_msgs::msg::CanBusData>::SharedPtr m_canbus_sub;

  /* 运行数据 */
  double m_engine_rpm = 0.0;
  int    m_current_gear = 1;   // 1=GEAR_DRIVE, 2=2挡 … 6

  /* 参数 */
  double max_accel;
  double max_decel;
};

}  // namespace autoware_lgsvl_interface

#endif  // AUTOWARE_LGSVL_INTERFACE__AUTOWARE_LGSVL_INTERFACE_HPP_