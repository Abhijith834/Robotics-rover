#!/usr/bin/env python3

"""
Robot Spawn Launch File

This launch file spawns the robot into an already running Gazebo simulation.
Useful for multi-robot scenarios or adding robots to existing simulations.

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
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name', 
        default_value='robot',
        description='Name of the robot to spawn'
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
    
    declare_robot_roll_arg = DeclareLaunchArgument(
        'robot_roll', 
        default_value='0.0',
        description='Initial robot roll orientation'
    )
    
    declare_robot_pitch_arg = DeclareLaunchArgument(
        'robot_pitch', 
        default_value='0.0',
        description='Initial robot pitch orientation'
    )
    
    declare_robot_yaw_arg = DeclareLaunchArgument(
        'robot_yaw', 
        default_value='1.5708',  # 90 degrees in radians
        description='Initial robot yaw orientation'
    )
    
    declare_model_path_arg = DeclareLaunchArgument(
        'model_path', 
        default_value=os.path.join(pkg_robot_simulation, 'models', 'robot', 'model.sdf'),
        description='Path to the robot SDF model file'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{LaunchConfiguration("robot_name")}',
        arguments=[
            '-file', LaunchConfiguration('model_path'),
            '-name', LaunchConfiguration('robot_name'),
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-R', LaunchConfiguration('robot_roll'),
            '-P', LaunchConfiguration('robot_pitch'),
            '-Y', LaunchConfiguration('robot_yaw')
        ],
        output='screen'
    )
    
    # Robot-specific state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{LaunchConfiguration("robot_name")}_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            os.path.join(pkg_robot_simulation, 'config', 'robot_params.yaml'),
            {'use_sim_time': True,
             'robot_description': open(os.path.join(pkg_robot_simulation, 'models', 'robot', 'model.sdf')).read()}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_robot_name_arg,
        declare_robot_x_arg,
        declare_robot_y_arg,
        declare_robot_z_arg,
        declare_robot_roll_arg,
        declare_robot_pitch_arg,
        declare_robot_yaw_arg,
        declare_model_path_arg,
        
        # Nodes
        spawn_robot,
        robot_state_publisher,
    ])