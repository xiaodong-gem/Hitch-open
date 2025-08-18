#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lgsvl_msgs.msg import VehicleControlData
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from race_msgs.msg import VehicleFlag
from pynput import keyboard
import threading
import csv
import os
from datetime import datetime
from tf_transformations import euler_from_quaternion
import pymap3d

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_z', 0.0)

        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.origin_z = self.get_parameter('origin_z').value
        
        # Declare parameters
        self.declare_parameter('save_waypoints', True)
        self.declare_parameter('output_dir', "./src/control/keyboard_controller/collected_waypoints")
        self.declare_parameter('flag_timeout', 0.2)  # Timeout in seconds
        
        self.save_waypoints_enabled = self.get_parameter('save_waypoints').value
        self.output_dir = self.get_parameter('output_dir').value
        self.flag_timeout = self.get_parameter('flag_timeout').value
        
        # Create publisher for vehicle control
        self.publisher = self.create_publisher(VehicleControlData, '/lgsvl/control', 10)   
        
        # Create subscriber for GPS data
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps_top/fix',
            self.gps_callback,
            10
        )

        # Create subscriber for vehicle flag
        self.vehicle_flag_sub = self.create_subscription(
            VehicleFlag,
            '/vehicle_flag',
            self.vehicle_flag_callback,
            10
        )
    
        self.latest_gps = None
        self.latest_odom = None
        self.waypoints = []  # List to store waypoints
        self.first_waypoint_recorded = False
        
        # Initialize control values
        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0
        self.current_gear = VehicleControlData.GEAR_DRIVE
        
        # Initialize flag state
        self.vehicle_flag = VehicleFlag.RED
        self.last_flag_time = self.get_clock().now()
        self._flag_to_string = {
            VehicleFlag.GREEN: 'GREEN',
            VehicleFlag.RED: 'RED',
            VehicleFlag.BLACK: 'BLACK',
            VehicleFlag.G10: 'G10',
            VehicleFlag.G20: 'G20',
            VehicleFlag.G40: 'G40',
            VehicleFlag.G60: 'G60',
            VehicleFlag.G80: 'G80',
            VehicleFlag.G100: 'G100'
        }
        
        # Control parameters
        self.throttle_step = 0.1
        self.brake_step = 0.1
        self.steering_step = 0.02
        self.max_steering = 0.15
        
        # Create timers for publishing control messages and printing GPS
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz for control
        self.flag_check_timer = self.create_timer(0.1, self.check_flag_timeout)  # 10Hz for flag checking

        # Start keyboard listener in a separate thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Print instructions
        self.get_logger().info("Keyboard Controller Started")
        self.get_logger().info(f"Current flag state: {self._flag_to_string[self.vehicle_flag]}")
        self.get_logger().info(f"Flag timeout: {self.flag_timeout} seconds")
        self.get_logger().info(f"Waypoint saving is {'enabled' if self.save_waypoints_enabled else 'disabled'}")
        if self.save_waypoints_enabled:
            self.get_logger().info(f"Waypoints will be saved to: {self.output_dir}")
        self.get_logger().info("Controls:")
        self.get_logger().info("W: Accelerate (only works with GREEN flags: GREEN, G10, G20, G40, G60, G80, G100)")
        self.get_logger().info("S: Brake")
        self.get_logger().info("A/D: Steer Left/Right")
        self.get_logger().info("↑: Shift up gear (only works with GREEN flags)")
        self.get_logger().info("↓: Shift down gear (only works with GREEN flags)")
        self.get_logger().info("Space: Record waypoint")
        self.get_logger().info("O: Save waypoints")
        self.get_logger().info("Q: Quit")

    def check_flag_timeout(self):
        """Check if we haven't received a flag message recently"""
        current_time = self.get_clock().now()
        time_since_last_flag = (current_time - self.last_flag_time).nanoseconds / 1e9
        
        if time_since_last_flag > self.flag_timeout :
            self.get_logger().warn(f'No flag received for {time_since_last_flag:.1f} seconds, defaulting to RED')
            self.vehicle_flag = VehicleFlag.RED
            # Reset control inputs
            self.throttle = 0.0
            self.brake = 1.0
            self.steering = 0.0

    def gps_callback(self, msg):
        """Callback for GPS data"""
        self.latest_gps = msg
        # Record first waypoint when we get first GPS reading
        if not self.first_waypoint_recorded and self.latest_odom is not None:
            self.record_waypoint()
            self.first_waypoint_recorded = True

    def odom_callback(self, msg):
        """Callback for odometry data"""
        self.latest_odom = msg
        # Record first waypoint when we get first odometry reading
        if not self.first_waypoint_recorded and self.latest_gps is not None:
            self.record_waypoint()
            self.first_waypoint_recorded = True

    def record_waypoint(self):

        if self.latest_gps is not None:
            # 直接保存原始GPS坐标
            waypoint = (
                self.latest_gps.latitude,
                self.latest_gps.longitude,
                self.latest_gps.altitude,
                0.0,  # 占位符1
                0.0   # 占位符2
            )
            self.waypoints.append(waypoint)
            
            # 计算ENU坐标用于日志显示（可选）
            """ local_x, local_y, local_z = pymap3d.geodetic2enu(
                self.latest_gps.latitude, 
                self.latest_gps.longitude, 
                self.latest_gps.altitude, 
                self.origin_x, self.origin_y, self.origin_z
            ) """
            
            self.get_logger().info(
                f'Recorded waypoint {len(self.waypoints)}: '
                f'Lat: {self.latest_gps.latitude:.6f}, '
                f'Lon: {self.latest_gps.longitude:.6f}, '
                f'Alt: {self.latest_gps.altitude:.2f}, '
                )
        """ f'X: {local_x:.2f}, '  # 显示ENU坐标（可选）
        f'Y: {local_y:.2f}, '  # 显示ENU坐标（可选）
        f'Z: {local_z:.2f}'    # 显示ENU坐标（可选 ）"""
    

    def save_waypoints(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints to save')
            return
            
        # Create filename with timestamp           哦
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'waypoints_{timestamp}.csv'
        
        # Ensure the directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            # Write waypoints to CSV
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write waypoints
                writer.writerows(self.waypoints)
                
            self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {str(e)}')

    def vehicle_flag_callback(self, msg):
        """Callback for vehicle flag updates"""
        old_flag = self.vehicle_flag
        self.vehicle_flag = msg.flag
        self.last_flag_time = self.get_clock().now()
        if old_flag != self.vehicle_flag:
            self.get_logger().info(f'Vehicle flag changed to: {self._flag_to_string[self.vehicle_flag]}')

    def keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def shift_up(self):
        if self.current_gear < 6:
            self.current_gear += 1
            self.get_logger().info(f'Shifted up to gear {self.current_gear}')
        else:
            self.get_logger().info(f'Already on max gear {self.current_gear}')

    def shift_down(self):
        if self.current_gear > 0:
            self.current_gear -= 1
            self.get_logger().info(f'Shifted down to gear {self.current_gear}')
        else:
            self.get_logger().info(f'Already on min gear {self.current_gear}')

    def on_press(self, key):
        try:
            if key.char == '8':
                if (self.vehicle_flag == VehicleFlag.GREEN or 
                    self.vehicle_flag == VehicleFlag.G10 or
                    self.vehicle_flag == VehicleFlag.G20 or
                    self.vehicle_flag == VehicleFlag.G40 or
                    self.vehicle_flag == VehicleFlag.G60 or
                    self.vehicle_flag == VehicleFlag.G80 or
                    self.vehicle_flag == VehicleFlag.G100):
                    self.throttle = min(0.3, self.throttle + self.throttle_step)
                    self.brake = 0.0
                else:
                    self.get_logger().warn(f'Cannot accelerate: Flag is {self._flag_to_string[self.vehicle_flag]}')
            elif key.char == '5':
                self.brake = min(1.0, self.brake + self.brake_step)
                self.throttle = 0.0
            elif key.char == '4':
                self.steering = min(self.max_steering, self.steering - self.steering_step)
            elif key.char == '6':
                self.steering = max(-self.max_steering, self.steering + self.steering_step)
            elif key.char == 'o':
                if self.save_waypoints_enabled:
                    self.get_logger().info('Saving waypoints...')
                    self.save_waypoints()
                else:
                    self.get_logger().info('Waypoints recording is disabled')
            elif key.char == 'q':
                self.get_logger().info('Shutting down...')
                rclpy.shutdown()
        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.up:
                if (self.vehicle_flag == VehicleFlag.GREEN or 
                    self.vehicle_flag == VehicleFlag.G10 or
                    self.vehicle_flag == VehicleFlag.G20 or
                    self.vehicle_flag == VehicleFlag.G40 or
                    self.vehicle_flag == VehicleFlag.G60 or
                    self.vehicle_flag == VehicleFlag.G80 or
                    self.vehicle_flag == VehicleFlag.G100):
                    self.shift_up()
                else:
                    self.get_logger().warn(f'Cannot shift: Flag is {self._flag_to_string[self.vehicle_flag]}')
            elif key == keyboard.Key.down:
                if (self.vehicle_flag == VehicleFlag.GREEN or 
                    self.vehicle_flag == VehicleFlag.G10 or
                    self.vehicle_flag == VehicleFlag.G20 or
                    self.vehicle_flag == VehicleFlag.G40 or
                    self.vehicle_flag == VehicleFlag.G60 or
                    self.vehicle_flag == VehicleFlag.G80 or
                    self.vehicle_flag == VehicleFlag.G100):
                    self.shift_down()
                else:
                    self.get_logger().warn(f'Cannot shift: Flag is {self._flag_to_string[self.vehicle_flag]}')
            elif key == keyboard.Key.ctrl_r:
                if self.save_waypoints_enabled:
                    self.record_waypoint()
                else:
                    self.get_logger().info('Waypoints recording is disabled')

    def on_release(self, key):
        try:
            if key.char == '8':
                self.throttle = 0.0
            elif key.char == '5':
                self.brake = 0.0
            elif key.char in ['4', '6']:
                self.steering = 0.0
        except AttributeError:
            pass

    def timer_callback(self):
        msg = VehicleControlData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
    # Set control values

        msg.acceleration_pct = float(self.throttle)
        msg.braking_pct = float(self.brake)
        msg.target_wheel_angle = float(self.steering)
        if self.vehicle_flag == VehicleFlag.RED:
            msg.braking_pct = 1.0  
            
        msg.target_wheel_angular_rate = 0.0
        msg.target_gear = self.current_gear

        # Publish the control message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save waypoints on keyboard interrupt
        node.save_waypoints()
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
