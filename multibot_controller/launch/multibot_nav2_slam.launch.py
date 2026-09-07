"""Run the TurtleBot4 simulation with SLAM, Nav2, and congestion costmaps.

Nav2 owns /robotN/cmd_vel in this launch. The legacy APF controller is disabled
to prevent two controllers from publishing conflicting velocity commands.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NUM_ROBOTS = 5
NAV2_DELAY = 5.0


def nav2_nodes(namespace, use_sim_time, params_path):
    """Create one non-composed Nav2 stack while keeping TF on global /tf."""
    # Load YAML and do dict-based parameter substitution (more reliable than ParameterFile)
    with open(params_path, 'r') as f:
        nav2_params = yaml.safe_load(f)
    
    def replace_namespace(obj, old_val, new_val):
        """Recursively replace namespace placeholder in all values."""
        if isinstance(obj, dict):
            return {k: replace_namespace(v, old_val, new_val) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [replace_namespace(item, old_val, new_val) for item in obj]
        elif isinstance(obj, str):
            return obj.replace(old_val, new_val)
        return obj
    
    nav2_params = replace_namespace(nav2_params, '<robot_namespace>', f'/{namespace}')
    
    # Pass parameters as Python dict, which ROS processes correctly
    configured_params = nav2_params

    common = {
        'namespace': namespace,
        'output': 'screen',
        'parameters': [configured_params],
    }
    return [
        Node(package='nav2_controller', executable='controller_server',
             remappings=[('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_smoother', executable='smoother_server', **common),
        Node(package='nav2_planner', executable='planner_server', **common),
        Node(package='nav2_behaviors', executable='behavior_server', **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')], **common),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server', 'smoother_server', 'planner_server',
                    'behavior_server', 'bt_navigator', 'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_dir = get_package_share_directory('multibot_controller')
    base_launch = os.path.join(package_dir, 'launch', 'multibot_10robots_ignition.launch.py')
    nav2_params = os.path.join(package_dir, 'config', 'nav2_slam.yaml')
    robot_namespaces = [f'robot{i}' for i in range(1, NUM_ROBOTS + 1)]

    launch = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Fast DDS shared-memory ports can remain locked after a large
        # multi-process launch. UDP avoids that transport-level race while
        # remaining sufficient for five local simulated robots.
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_multimode_controller': 'false',
            }.items()),
        Node(
            package='multibot_controller',
            executable='congestion_zone_manager_node',
            name='congestion_zone_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='multibot_controller',
            executable='congestion_costmap.py',
            name='congestion_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_namespaces': robot_namespaces,
                'zone_padding': 0.35,
            }],
        ),
    ])

    for namespace in robot_namespaces:
        launch.add_action(TimerAction(
            period=NAV2_DELAY,
            actions=nav2_nodes(namespace, use_sim_time, nav2_params)))

    return launch
