"""
multibot_10robots_ignition.launch.py

Spawn 10 robot vao Gazebo Sim (Fortress) trong world warehouse.
Moi robot: robot_state_publisher + spawn + bridge + controller_manager spawner.
MultimodeController quan ly ca 10 robot cung luc.

Chay:
  ros2 launch multibot_controller multibot_10robots_ignition.launch.py

Switch controllet 1 robot bat ky:
  ros2 service call /multimode_controller/switch_controllet \
    multibot_interfaces/srv/SwitchControllet \
    "{robot_namespace: 'robot3', controllet_name: 'patrol'}"
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
NUM_ROBOTS = 1
SPAWN_INTERVAL = 6.0     # giay giua moi lan spawn robot
CONTROLLER_DELAY = 20.0   # giay cho sau spawn truoc khi goi controller_manager spawner

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument(
        'enable_multimode_controller', default_value='true',
        description='Start the APF multimode controller. Set false when Nav2 owns cmd_vel.'),
]


def get_spawn_pose(robot_id, cols= 5):
    """2 hang x 10 cot, trong VUNG ROBOT rieng (x:[23.6,29.6], y:[0,6])."""
    row = robot_id // cols
    col = robot_id % cols
    x = 26.6 + (col - (cols - 1) / 2.0) * 1.0
    y = 3.0 + (row - 0.5) * 2.0
    return x, y, 0.05

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_multimode_controller = LaunchConfiguration('enable_multimode_controller')

    turtlebot_description_dir = get_package_share_directory('turtlebot4_description')
    multibot_controller_dir = get_package_share_directory('multibot_controller')

    world_path = os.path.join(multibot_controller_dir, 'worlds', 'warehouse_layout2.world')

    # =========================================================================
    # GAZEBO SIM
    # -s: server-only (khong GUI, do tai may)
    # --render-engine ogre2: tranh crash Ogre1 khi tao nhieu light/sensor cung luc
    # =========================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r --render-engine ogre2 {world_path}'
        }.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(clock_bridge)

    robot_description_path = PathJoinSubstitution([
        turtlebot_description_dir,
        'urdf','standard',
        'turtlebot4.urdf.xacro'
    ])

    robot_namespaces = []

    for i in range(1, NUM_ROBOTS + 1):
        ns = f'robot{i}'
        robot_namespaces.append(ns)
        x, y, z = get_spawn_pose(i - 1)

        robot_desc = Command([
            'xacro ', robot_description_path,
            ' gazebo:=ignition',
            f' namespace:={ns}'
        ])

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_desc, value_type=str),
                'use_sim_time': use_sim_time,
                'frame_prefix': f'{ns}/'
            }]
        )

        # Stagger spawn theo thoi gian de tranh dung vat ly luc khoi tao cung luc
        spawn_delay = SPAWN_INTERVAL * i
        spawn = TimerAction(
            period=spawn_delay,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', f'turtlebot4_{i}',
                    '-topic', f'/{ns}/robot_description',
                    '-x', str(x), '-y', str(y), '-z', str(z), '-Y', '0.0'
                ],
                output='screen'
            )]
        )

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=ns,
            arguments=[
                f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'/{ns}/odom_gt@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )

        # Controller config (controllers.yaml) duoc nap qua xacro <ros2_control>/<gazebo>
        # plugin tag, khong can truyen lai o day.
        

        controller_spawners = TimerAction(
            period=spawn_delay + CONTROLLER_DELAY,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '-c', f'/{ns}/controller_manager', '--controller-manager-timeout', '60'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['wheel_velocity_controller', '-c', f'/{ns}/controller_manager', '--controller-manager-timeout', '60'],
                    output='screen'
                ),
            ]
        )

        twist_to_wheels = Node(
            package='multibot_controller',
            executable='twist_to_wheels.py',
            name=f'twist_to_wheels_{ns}',
            namespace=ns,
            output='screen'
        )
        wheel_odom = Node(
            package='multibot_controller',
            executable='wheel_odom.py',
            name=f'wheel_odom_{ns}',
            namespace=ns,
            output='screen',
            parameters=[{'start_x': x, 'start_y': y, 'use_sim_time': use_sim_time}]
        )
        ekf_config = os.path.join(multibot_controller_dir, 'config', 'ekf.yaml')
        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name=f'ekf_filter_node_{ns}',
            namespace=ns,
            output='screen',
            parameters=[
                ekf_config,
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame': f'{ns}/odom',
                    'base_link_frame': f'{ns}/base_link',
                    'world_frame': f'{ns}/odom',
                }
            ],
            remappings=[
                ('odom', 'odom'),
                ('imu', 'imu'),
                ('odometry/filtered', 'odometry/filtered'),
            ]
        )
        fix_scan = Node(
            package='multibot_controller',
            executable='fix_scan_frame.py',
            name=f'fix_scan_{ns}',
            namespace=ns,
            output='screen'
        )
        slam = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            namespace=ns,
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': f'{ns}/odom',
                'map_frame': f'{ns}/map',
                'base_frame': f'{ns}/base_link',
                'scan_topic': f'/{ns}/scan_fixed',
                'resolution': 0.05,
                'max_laser_range': 12.0,
            }],
            remappings=[
                ('/map', f'/{ns}/map'),
                ('/map_metadata', f'/{ns}/map_metadata'),
            ],
        )
        ld.add_action(slam)
        
        ld.add_action(rsp)
        ld.add_action(spawn)
        ld.add_action(bridge)
        ld.add_action(controller_spawners)
        ld.add_action(twist_to_wheels)
        ld.add_action(wheel_odom)
        ld.add_action(ekf_node)
        ld.add_action(fix_scan)
    # =========================================================================
    # MULTIMODE CONTROLLER - quan ly ca 10 robot
    # Start sau khi robot cuoi (robot10) da spawn + controller da len xong
    # =========================================================================
    last_robot_ready_time = SPAWN_INTERVAL * NUM_ROBOTS + CONTROLLER_DELAY
    multimode_controller = TimerAction(
        period=last_robot_ready_time + 10.0,
        condition=IfCondition(enable_multimode_controller),
        actions=[Node(
            package='multibot_controller',
            executable='multimode_controller_node',
            name='multimode_controller',
            output='screen',
            parameters=[
                os.path.join(multibot_controller_dir, 'config', 'controllers.yaml'),
                {'use_sim_time': use_sim_time},
                {'robot_namespaces': robot_namespaces}
            ]
        )]
    )
    ld.add_action(multimode_controller)

    return ld
