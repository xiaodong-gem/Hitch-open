#!/usr/bin/env python3
import time
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from lgsvl_msgs.msg import VehicleControlData

def main():
    rclpy.init()
    node = rclpy.create_node('auto_gear_3')

    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )
    pub = node.create_publisher(VehicleControlData, '/lgsvl/control', qos)

    def send_gear(gear):
        msg = VehicleControlData()
        msg.target_gear = gear
        msg.acceleration_pct = 0.0
        msg.braking_pct = 0.0
        pub.publish(msg)

    current_gear = 0
    target_gear = 4

    while current_gear < target_gear:
        current_gear += 1
        node.get_logger().info(f'换档 -> {current_gear}')
        # 以 50 Hz 持续 1 秒发送，确保 LGSVL 收到
        for _ in range(50):
            send_gear(current_gear)
            time.sleep(0.02)
        time.sleep(0.2)   # 稍作间隔再升下一档

    node.get_logger().info('已到达 6 档，脚本结束。')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()