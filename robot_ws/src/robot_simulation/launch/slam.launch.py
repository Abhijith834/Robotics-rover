import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_robot_simulation = get_package_share_directory('robot_simulation')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz for visualization')
    
    # Include the robot simulation
    robot_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_simulation'),
                'launch',
                'robot_simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'teleop': 'true',
            'rviz': 'false'  # We'll launch our own RViz with SLAM config
        }.items()
    )
    
    # SLAM Toolbox Node - using front LiDAR only (more reliable)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/lidar_front/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 10.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'do_loop_closing': True,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_fine': 0.45,
            'loop_match_maximum_variance_fine': 0.2,
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1
        }],
        remappings=[
            ('/odom', '/model/robot/odometry')
        ]
    )
    
    # RViz for SLAM visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz)
    
    # Start robot simulation
    ld.add_action(robot_simulation_launch)
    
    # Start SLAM after simulation is ready
    ld.add_action(TimerAction(period=8.0, actions=[slam_toolbox_node]))
    
    # Start visualization
    ld.add_action(TimerAction(period=10.0, actions=[rviz_node]))
    
    return ld
