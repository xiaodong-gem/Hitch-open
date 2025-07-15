# Copyright 2024 AI Racing Tech

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from base_common import get_sim_time_launch_arg

import os
from environs import Env

base_remappings = [
    ("vehicle_control_cmd", "/auto/raw_command"),
    ("can_bus", "/can"),
    ("lgsvl_odom", "/lgsvl/odom"),
    ("imu", "/gps_bot/imu"),
    ("odom", "/odom"),
    ("lgsvl_control", "/lgsvl/control"),
    ("gps_fix", "/gps_bot/gps"),
    ("steering_report", "/vehicle/steering_report"),
    ("lgsvl_kinematic_state", "/lgsvl/state"),
    ("fix", "/gps_bot/fix"),
    ("engine_report", "/vehicle/engine_report"),
    ("wheel_speeds", "/vehicle/wheel_speed_report"),
    ("low_level_fault_report", "/vehicle/low_level_fault_report"),
]

lidar_remappings = [
    ("pointcloud_in_0", "/lgsvl/luminar_front_points"),
    ("pointcloud_in_1", "/lgsvl/luminar_left_points"),
    ("pointcloud_in_2", "/lgsvl/luminar_right_points"),
    ("pointcloud_out_0", "/luminar_front_points"),
    ("pointcloud_out_1", "/luminar_left_points"),
    ("pointcloud_out_2", "/luminar_right_points"),
]

camera_remappings = [
    ("compressed_image_in_0", "/vimba_front_left/image/compressed"),
    ("compressed_image_in_1", "/vimba_front_left_center/image/compressed"),
    ("compressed_image_in_2", "/vimba_front_right_center/image/compressed"),
    ("compressed_image_in_3", "/vimba_front_right/image/compressed"),
    ("image_out_0", "/vimba_front_left/image"),
    ("image_out_1", "/vimba_front_left_center/image"),
    ("image_out_2", "/vimba_front_right_center/image"),
    ("image_out_3", "/vimba_front_right/image"),
]
full_camera_remappings = [
    ("compressed_image_in_4", "/vimba_rear_left/image/compressed"),
    ("compressed_image_in_5", "/vimba_rear_right/image/compressed"),
    ("image_out_4", "/vimba_rear_left/image"),
    ("image_out_5", "/vimba_rear_right/image"),
]

env = Env()
env.read_env("race.env")

SENSORS = env.str("SENSORS", "GPS_LIDAR")

num_lidars = 0
if "LIDAR" in SENSORS:
    base_remappings += lidar_remappings
    num_lidars = 3

num_cameras = 0
if "CAMERAS" in SENSORS:
    base_remappings += camera_remappings
    num_cameras = 4
if "CAMERAS_FULL" in SENSORS:
    base_remappings += full_camera_remappings
    num_cameras = 6


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    lgsvl_interface_param = os.path.join(
        get_package_share_directory("lgsvl_interface"),
        "param",
        "lgsvl_interface.param.yaml",
    )
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            Node(
                package="lgsvl_interface",
                executable="lgsvl_interface_node_exe",
                parameters=[
                    lgsvl_interface_param,
                    use_sim_time,
                    {
                        "origin.latitude": env.float("GPS_ORIGIN_LAT"),
                        "origin.longitude": env.float("GPS_ORIGIN_LON"),
                        "origin.altitude": env.float("GPS_ORIGIN_ALT"),
                        "num_pointclouds": num_lidars,
                        "num_cameras": num_cameras,
                    },
                ],
                remappings=base_remappings,
            ),
        ]
    )
