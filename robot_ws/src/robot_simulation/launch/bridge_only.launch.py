#!/usr/bin/env python3

"""
Bridge Only Launch File

This launch file starts only the ROS-Gazebo bridge for connecting
to an already running Gazebo simulation. Useful for testing and debugging
the bridge configuration independently.

Author: Your Name
License: Apache-2.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_robot_simulation = get_package_share_directory('robot_simulation')
    
    # Declare launch arguments
    declare_config_arg = DeclareLaunchArgument(
        'bridge_config', 
        default_value=os.path.join(pkg_robot_simulation, 'config', 'bridge_config.yaml'),
        description='Path to the bridge configuration file'
    )
    
    declare_verbose_arg = DeclareLaunchArgument(
        'verbose', 
        default_value='false',
        description='Enable verbose output'
    )
    
    # ROS-Gazebo bridge node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': LaunchConfiguration('bridge_config'),
            'use_sim_time': True
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # Robot state publisher for TF frames
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            os.path.join(pkg_robot_simulation, 'config', 'robot_params.yaml'),
            {'use_sim_time': True}
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
    
    return LaunchDescription([
        # Launch arguments
        declare_config_arg,
        declare_verbose_arg,
        
        # Nodes
        bridge_node,
        robot_state_publisher,
        joint_state_publisher,
    ])