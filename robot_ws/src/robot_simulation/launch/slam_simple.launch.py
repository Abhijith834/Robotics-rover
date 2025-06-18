from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Base simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_simulation'),
            '/launch/robot_simulation.launch.py'
        ]),
        launch_arguments={
            'teleop': 'true',
            'rviz': 'false'
        }.items()
    )

    # SLAM Toolbox with CORRECT topic and frame remapping
    slam = TimerAction(
        period=8.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_link',  # ✅ Match your robot's frame
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',  # ✅ Will be remapped from /lidar_front/scan
                'max_laser_range': 10.0,
                'resolution': 0.05,
                'transform_timeout': 0.2,
                'mode': 'mapping',
                'minimum_time_interval': 0.5,
                'do_loop_closing': True,
                'minimum_travel_distance': 0.5,
                'minimum_travel_heading': 0.5
            }],
            remappings=[
                ('/scan', '/lidar_front/scan'),        # ✅ CRITICAL: Map lidar topic to expected /scan
                ('/odom', '/model/robot/odometry'),     # ✅ CRITICAL: Map odometry topic
                ('/map', '/map')                        # ✅ Ensure map topic is correct
            ]
        )]
    )

    # RViz for visualization
    rviz = TimerAction(
        period=10.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return LaunchDescription([
        simulation,
        slam,
        rviz
    ])
