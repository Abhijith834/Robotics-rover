import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_simulation')
    world_file = os.path.join(pkg_dir, 'worlds', 'test_world.sdf')
    model_file = os.path.join(pkg_dir, 'models', 'robot', 'model.sdf')
    
    return LaunchDescription([
        # Environment
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.pathsep.join([
                pkg_dir,
                os.path.join(pkg_dir, 'models'),
                os.path.join(pkg_dir, 'worlds')
            ])
        ),
        
        # Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file],
            output='screen'
        ),
        
        # Robot State Publisher (from your working config)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': Command(['cat ', model_file])}
            ]
        ),
        
        # Joint State Publisher (from your working config)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Robot spawn
        TimerAction(
            period=3.0,
            actions=[ExecuteProcess(
                cmd=[
                    'ign', 'service', '-s', '/world/test_world/create',
                    '--reqtype', 'ignition.msgs.EntityFactory',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', f'sdf_filename: "{model_file}"'
                ],
                output='screen'
            )]
        ),
        
        # YOUR WORKING BRIDGE CONFIGURATION
        TimerAction(
            period=5.0,
            actions=[
                # Controller Bridge (your working config)
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
                        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/model/robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
                    ],
                    output='screen',
                    name='controller_bridge'
                ),
                # LiDAR Bridge (your working config)
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
                        '/lidar_front@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                        '/lidar_back@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
                    ],
                    output='screen',
                    name='lidar_bridge'
                ),
                # Camera Bridge (your working config)
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
                        '/world/test_world/model/robot/model/camera/link/link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                        '/world/test_world/model/robot/model/camera/link/link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
                    ],
                    output='screen',
                    name='camera_bridge'
                ),
                # Odometry Bridge (your working config)
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
                        '/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
                    ],
                    output='screen',
                    name='odom_bridge'
                ),
                # Joint States Bridge (your working config)
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
                        '/world/test_world/model/robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
                    ],
                    output='screen',
                    name='joint_states_bridge'
                )
            ]
        ),
        
        # YOUR WORKING TRANSFORM CONFIGURATION
        TimerAction(
            period=6.0,
            actions=[
                # Map to Odom Transform
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                        '0', '0', '0', '0', '0', '0',
                        'map', 'odom'
                    ],
                    output='screen',
                    name='map_to_odom_tf'
                ),
                # Odom to Base Link Transform
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                        '0', '0', '0', '0', '0', '0',
                        'odom', 'base_link'
                    ],
                    output='screen',
                    name='odom_to_base_link_tf'
                ),
                # LiDAR Front Transform
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                        '0', '0', '0', '0', '0', '0',
                        'base_link', 'robot/link/lidar_front'
                    ],
                    output='screen',
                    name='lidar_front_tf'
                ),
                # LiDAR Back Transform
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'tf2_ros', 'static_transform_publisher', 
                        '0', '0', '0', '0', '0', '0',
                        'base_link', 'robot/link/lidar_back'
                    ],
                    output='screen',
                    name='lidar_back_tf'
                )
            ]
        ),
        
        # SLAM Toolbox
        TimerAction(
            period=8.0,
            actions=[Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'base_frame': 'base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'scan_topic': '/lidar_front',  # Using your working topic name
                    'mode': 'mapping',
                    'max_laser_range': 10.0,
                    'resolution': 0.05,
                    'minimum_travel_distance': 0.2,
                    'minimum_travel_heading': 0.1,
                    'transform_timeout': 0.2,
                    'do_loop_closing': True
                }],
                remappings=[
                    ('/odom', '/model/robot/odometry')
                ]
            )]
        ),
        
        # Teleop (your working config)
        TimerAction(
            period=9.0,
            actions=[ExecuteProcess(
                cmd=[
                    'xterm', '-e', 
                    'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                    '--ros-args', '--remap', 'cmd_vel:=/cmd_vel'
                ],
                output='screen',
                name='teleop_keyboard'
            )]
        ),
        
        # RViz2 with config file
        TimerAction(
            period=10.0,
            actions=[Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{
                    'use_sim_time': True
                }],
                arguments=['-d', os.path.join(pkg_dir, 'config', 'slam_rviz.rviz')]
            )]
        )
    ])
