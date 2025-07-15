from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from environs import Env
import yaml

env = Env()
env.read_env("race.env")

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('competition_timer')
    
    # Load competition parameters
    competition_yaml = os.path.join(pkg_share, 'config', 'competition.yaml')
    with open(competition_yaml, 'r') as f:
        config = yaml.safe_load(f)
    
    map_name = env.str("MAP_NAME")
    map_config_path = os.path.join(pkg_share, 'config', f'{map_name}.yaml')

    # Competition timer node
    timer_node = Node(
        package='competition_timer',
        executable='competition_timer',
        name='competition_timer_node',
        output='screen',
        parameters=[{
            "results_dir": config['results_dir'],
            "use_sim_time": env.bool("USE_SIM_TIME"),
            "config_file_path": map_config_path,
            "target_laps": config['target_laps'],
            "vehicle_flag": config['vehicle_flag']
        }]
    )

    # Include misc.launch.py
    misc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('autonomy_launch'), 'launch', 'misc.launch.py')
        ])
    )

    # Include lgsvl_interface.launch.py
    lgsvl_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lgsvl_interface'), 'launch', 'lgsvl_interface.launch.py')
        ])
    )

    return LaunchDescription([
        misc_launch,
        lgsvl_interface_launch,
        timer_node
    ]) 