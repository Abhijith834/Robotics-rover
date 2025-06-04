# Bridge-Only Launch File with Manual Parameter Bridge Commands
# This launch file starts only the bridge components for testing

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package and file paths
    pkg_robot_simulation = get_package_share_directory('robot_simulation')
    
    # Launch arguments
    declare_verbose_cmd = DeclareLaunchArgument(
        'verbose', 
        default_value='false',
        description='Set to "true" for verbose output'
    )
    
    declare_teleop_cmd = DeclareLaunchArgument(
        'teleop', 
        default_value='true',
        description='Start teleoperation'
    )

    # Controller Bridge - for cmd_vel control
    controller_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ],
        output='screen',
        name='controller_bridge'
    )

    # LiDAR Bridge - for both front and back LiDAR sensors
    lidar_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
            '/lidar_front@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/lidar_back@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen',
        name='lidar_bridge'
    )

    # Camera Bridge - for camera image and info
    camera_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
            '/world/test_world/model/robot/model/camera/link/link/sensor/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/test_world/model/robot/model/camera/link/link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        output='screen',
        name='camera_bridge'
    )

    # Odometry Bridge
    odom_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
            '/model/robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
        ],
        output='screen',
        name='odom_bridge'
    )

    # Joint States Bridge
    joint_states_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
            '/world/test_world/model/robot/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model'
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

    # Standard Teleoperation
    teleop_keyboard = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', '--remap', 'cmd_vel:=/cmd_vel'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop')),
        name='teleop_keyboard'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_verbose_cmd)
    ld.add_action(declare_teleop_cmd)

    # Add bridge components
    ld.add_action(controller_bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(camera_bridge)
    ld.add_action(odom_bridge)
    ld.add_action(joint_states_bridge)
    
    # Add transform publishers
    ld.add_action(lidar_front_tf)
    ld.add_action(lidar_back_tf)
    
    # Add teleoperation
    ld.add_action(teleop_keyboard)

    return ld