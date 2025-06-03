from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), 
         glob(os.path.join('worlds', '*.sdf'))),
        # Install robot model files
        (os.path.join('share', package_name, 'models', 'robot'), 
         glob(os.path.join('models', 'robot', '*'))),
        # Install scripts directory
        (os.path.join('share', package_name, 'scripts'), 
         glob(os.path.join('scripts', '*.py'))),
        # Install CMakeLists.txt
        ('share/' + package_name, ['CMakeLists.txt']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'sensor_msgs',
        'nav_msgs',
        'std_msgs',
        'diagnostic_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
        'pynput',
        'termios',
        'select',
        'sys',
        'tty',
        'threading',
        'time',
        'math',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Complete ROS 2 simulation package for a dual LiDAR robot with camera and differential drive',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = robot_simulation.teleop_keyboard:main',
            'robot_monitor = robot_simulation.robot_monitor:main',
        ],
    },
)