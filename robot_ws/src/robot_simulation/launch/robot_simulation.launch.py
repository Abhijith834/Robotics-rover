#!/usr/bin/env python3

"""
Robot Simulation Launch File

This launch file starts the complete robot simulation including:
- Gazebo with the test world
- Robot spawning 
- ROS-Gazebo bridge for sensor data
- Robot state publisher
- Optional RViz visualization
- Optional teleoperation

Author: Your Name
License: Apache-2.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_robot_simulation = get_package_share_directory('robot_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world', 
        default_value=os.path.join(pkg_robot_simulation, 'worlds', 'test_world.sdf'),
        description='Path to the Gazebo world file'
    )
    
    declare_gui_arg = DeclareLaunchArgument(
        'gui', 
        default_value='true',
        description='Set to false for headless mode'
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    declare_teleop_arg = DeclareLaunchArgument(
        'teleop', 
        default_value='false',
        description='Launch keyboard teleoperation'
    )
    
    declare_verbose_arg = DeclareLaunchArgument(
        'verbose', 
        default_value='false',
        description='Enable verbose output'
    )
    
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name', 
        default_value='robot',
        description='Name of the robot'
    )
    
    declare_robot_x_arg = DeclareLaunchArgument(
        'robot_x', 
        default_value='0.0',
        description='Initial robot X position'
    )
    
    declare_robot_y_arg = DeclareLaunchArgument(
        'robot_y', 
        default_value='0.0',
        description='Initial robot Y position'
    )
    
    declare_robot_z_arg = DeclareLaunchArgument(
        'robot_z', 
        default_value='0.5',
        description='Initial robot Z position'
    )
    
    declare_robot_yaw_arg = DeclareLaunchArgument(
        'robot_yaw', 
        default_value='1.5708',  # 90 degrees in radians
        description='Initial robot yaw orientation'
    )
    
    # Set environment variables for Gazebo
    set_env_vars = [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.pathsep.join([
                get_package_share_directory('robot_simulation'),
                os.path.join(get_package_share_directory('robot_simulation'), 'models')

            ])
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=os.pathsep.join([
                '/opt/ros/humble/lib',  # Adjust for your ROS distribution
                '/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins'  # Adjust for your Gazebo version
            ])
        )
    ]
    
    # Launch Gazebo with the world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -v 4' if LaunchConfiguration('verbose') else ''],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-file', os.path.join(pkg_robot_simulation, 'models', 'robot', 'model.sdf'),
            '-name', LaunchConfiguration('robot_name'),
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-Y', LaunchConfiguration('robot_yaw')
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_robot_simulation, 'config', 'bridge_config.yaml')
        }],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            os.path.join(pkg_robot_simulation, 'config', 'robot_params.yaml')
        ],
        output='screen'
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
)
    
    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Teleoperation (optional)
    teleop_node = Node(
        package='robot_simulation',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop'))
    )
    
    # Robot monitor
    monitor_node = Node(
        package='robot_simulation',
        executable='robot_monitor',
        name='robot_monitor',
        output='screen'
    )
    
    return LaunchDescription([
        # Environment variables
        *set_env_vars,
        
        # Launch arguments
        declare_world_arg,
        declare_gui_arg,
        declare_rviz_arg,
        declare_teleop_arg,
        declare_verbose_arg,
        declare_robot_name_arg,
        declare_robot_x_arg,
        declare_robot_y_arg,
        declare_robot_z_arg,
        declare_robot_yaw_arg,
        
        # Launch nodes
        gazebo_launch,
        spawn_robot,
        bridge_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        teleop_node,
        monitor_node,
    ])