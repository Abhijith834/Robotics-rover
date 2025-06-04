# Robot Spawn Launch File
# This launch file handles spawning the robot into an existing simulation

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package and file paths
    pkg_robot_simulation = get_package_share_directory('robot_simulation')
    model_file_path = os.path.join(pkg_robot_simulation, 'models', 'robot', 'model.sdf')
    
    # Launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', 
        default_value='robot',
        description='Name of the robot'
    )
    
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name', 
        default_value='test_world',
        description='Name of the world'
    )
    
    declare_x_cmd = DeclareLaunchArgument(
        'x', 
        default_value='0.0',
        description='X position'
    )
    
    declare_y_cmd = DeclareLaunchArgument(
        'y', 
        default_value='0.0',
        description='Y position'
    )
    
    declare_z_cmd = DeclareLaunchArgument(
        'z', 
        default_value='0.5',
        description='Z position'
    )
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', 
        default_value='1.57',
        description='Yaw rotation (Z-axis rotation)'
    )
    
    # Environment setup
    set_env_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            pkg_robot_simulation,
            os.path.join(pkg_robot_simulation, 'models')
        ])
    )
    
    # Robot spawning
    robot_spawn = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', 
            ['/world/', LaunchConfiguration('world_name'), '/create'],
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', [
                'sdf_filename: "', model_file_path, 
                '", pose: {position: {x: ', LaunchConfiguration('x'), 
                ', y: ', LaunchConfiguration('y'), 
                ', z: ', LaunchConfiguration('z'), 
                '}, orientation: {x: 0, y: 0, z: ', 
                LaunchConfiguration('yaw'), ', w: 1}}, name: "', 
                LaunchConfiguration('robot_name'), '"'
            ]
        ],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)
    
    # Environment setup
    ld.add_action(set_env_gz_resource_path)
    
    # Robot spawning
    ld.add_action(robot_spawn)
    
    return ld