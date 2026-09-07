"""Debug single controller_server to identify crash."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    package_dir = get_package_share_directory('multibot_controller')
    nav2_params_file = os.path.join(package_dir, 'config', 'nav2_slam.yaml')

    # Load and parse YAML
    with open(nav2_params_file, 'r') as f:
        nav2_params = yaml.safe_load(f)

    # Replace namespace placeholders
    def replace_namespace(obj, old_val, new_val):
        if isinstance(obj, dict):
            return {k: replace_namespace(v, old_val, new_val) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [replace_namespace(item, old_val, new_val) for item in obj]
        elif isinstance(obj, str):
            return obj.replace(old_val, new_val)
        return obj

    robot_name_str = 'robot1'  # Will be overridden by launch arg
    nav2_params = replace_namespace(nav2_params, '<robot_namespace>', f'/{robot_name_str}')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=robot_name,
            name='controller_server',
            output='screen',
            emulate_tty=True,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            parameters=[nav2_params],
        ),
    ])
