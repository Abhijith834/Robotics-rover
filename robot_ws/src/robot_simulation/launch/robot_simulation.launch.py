# Updated Robot Simulation Launch File with Manual Parameter Bridge Commands
# This launch file replaces YAML-based bridge with individual parameter_bridge nodes

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package and file paths
    pkg_robot_simulation = get_package_share_directory('robot_simulation')
    
    # World and model file paths - FIXED: Use actual paths
    world_file_path = os.path.join(pkg_robot_simulation, 'worlds', 'test_world.sdf')
    model_file_path = os.path.join(pkg_robot_simulation, 'models', 'robot', 'model.sdf')
    
    # Launch arguments
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file', 
        default_value=world_file_path,
        description='Full path to world file to load'
    )
    
    declare_gui_cmd = DeclareLaunchArgument(
        'gui', 
        default_value='true',
        description='Set to "false" to run headless'
    )
    
    declare_verbose_cmd = DeclareLaunchArgument(
        'verbose', 
        default_value='false',
        description='Set to "true" for verbose output'
    )
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', 
        default_value='robot',
        description='Name of the robot'
    )
    
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0') 
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.5')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='1.57')
    
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', 
        default_value='false',
        description='Start RViz'
    )
    
    declare_teleop_cmd = DeclareLaunchArgument(
        'teleop', 
        default_value='false',  # FIXED: Disabled by default for WSL
        description='Start teleoperation'
    )

    # Environment variables
    set_env_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            pkg_robot_simulation,
            os.path.join(pkg_robot_simulation, 'models'),
            os.path.join(pkg_robot_simulation, 'worlds')
        ])
    )

    # FIXED: Gazebo launch with proper file path
    gazebo_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    gazebo_headless_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file_path, '-s'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )


    # FIXED: Robot spawning with simplified request
    robot_spawn = ExecuteProcess(
        cmd=[
            'ign', 'service', '-s', '/world/test_world/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'sdf_filename: "{model_file_path}"'
        ],
        output='screen'
    )



    # Controller Bridge - for cmd_vel control
    controller_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',  # FIXED: Use ros_ign_bridge
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen',
        name='controller_bridge'
    )

    # LiDAR Bridge - for both front and back LiDAR sensors
    lidar_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',  # FIXED: Use ros_ign_bridge
            '/lidar_front@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar_back@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        output='screen',
        name='lidar_bridge'
    )

    # Camera Bridge - for camera image and info
    camera_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',  # FIXED: Use ros_ign_bridge
            '/world/test_world/model/robot/model/camera/link/link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/test_world/model/robot/model/camera/link/link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen',
        name='camera_bridge'
    )

    # Odometry Bridge
    odom_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',  # FIXED: Use ros_ign_bridge
            '/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen',
        name='odom_bridge'
    )

    # Joint States Bridge
    joint_states_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',  # FIXED: Use ros_ign_bridge
            '/world/test_world/model/robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen',
        name='joint_states_bridge'
    )

    # Static Transform Publishers for LiDAR sensors
    lidar_front_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0', '0', '0', '0', '0', '0',
            'map', 'robot/link/lidar_front'
        ],
        output='screen',
        name='lidar_front_tf'
    )

    lidar_back_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher', 
            '0', '0', '0', '0', '0', '0',
            'map', 'robot/link/lidar_back'
        ],
        output='screen',
        name='lidar_back_tf'
    )

    # FIXED: Robot State Publisher with proper robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['cat ', model_file_path])}  # FIXED: Added robot description
        ]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # REMOVED: Problematic teleop for WSL compatibility
    teleop_keyboard = ExecuteProcess(
        cmd=[
            'xterm', '-e', 
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', '--remap', 'cmd_vel:=/cmd_vel'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop')),
        name='teleop_keyboard'
    )

    # RViz
    rviz_config_file = os.path.join(pkg_robot_simulation, 'config', 'robot_sim.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # FIXED: Robot Monitor with better error handling
    robot_monitor = Node(
        package='robot_simulation',
        executable='robot_monitor.py',
        name='robot_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('verbose'))  # Only start if verbose mode
    )

    # Create launch description with timed actions
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_verbose_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_teleop_cmd)

    # Environment setup
    ld.add_action(set_env_gz_resource_path)

    # Start Gazebo
    ld.add_action(gazebo_launch)
    ld.add_action(gazebo_headless_launch)

    # Start robot state management (immediate)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    # Spawn robot after Gazebo starts (3 second delay)
    ld.add_action(TimerAction(period=3.0, actions=[robot_spawn]))

    # Start bridges after robot spawn (5 second delay)
    ld.add_action(TimerAction(period=5.0, actions=[
        controller_bridge,
        lidar_bridge,
        camera_bridge,
        odom_bridge,
        joint_states_bridge
    ]))

    # Start transform publishers (6 second delay)
    ld.add_action(TimerAction(period=6.0, actions=[
        lidar_front_tf,
        lidar_back_tf
    ]))

    # Start optional components (7 second delay)
    ld.add_action(TimerAction(period=7.0, actions=[
        teleop_keyboard,
        rviz,
        robot_monitor
    ]))

    return ld
