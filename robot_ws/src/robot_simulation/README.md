# Robot Simulation Package

A comprehensive ROS 2 simulation package for a dual LiDAR robot with camera and differential drive system. This package provides complete integration with Gazebo, sensor data bridging, teleoperation, and system monitoring capabilities.

## Table of Contents

- [Features](#features)
- [Package Structure](#package-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Nodes](#nodes)
- [Topics](#topics)
- [Launch Files](#launch-files)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## Features

### Robot Capabilities
- **Dual LiDAR System**: Front and rear-mounted LiDAR sensors with 640-sample resolution
- **Camera System**: Overhead camera with 320×240 resolution mounted on vertical pole
- **Differential Drive**: Four-wheel robot with front-wheel drive configuration
- **Physics Integration**: Realistic collision detection and physics simulation

### Software Features
- **Complete ROS 2 Integration**: Full bridge between Gazebo and ROS 2
- **Advanced Teleoperation**: Keyboard control with safety features and speed adjustment
- **System Monitoring**: Comprehensive diagnostics and health monitoring
- **Multi-Launch Support**: Flexible launch configurations for different scenarios
- **Safety Systems**: Emergency stop and collision avoidance capabilities

## Package Structure

```
robot_simulation/
├── package.xml                 # Package metadata and dependencies
├── setup.py                   # Python package setup
├── setup.cfg                  # Setup configuration
├── CMakeLists.txt             # Build configuration
├── resource/robot_simulation  # Resource marker file
├── README.md                  # This file
├── launch/                    # Launch files
│   ├── robot_simulation.launch.py     # Main simulation launcher
│   ├── bridge_only.launch.py          # Bridge-only mode
│   └── robot_spawn.launch.py          # Robot spawning utility
├── config/                    # Configuration files
│   ├── bridge_config.yaml             # ROS-Gazebo bridge configuration
│   └── robot_params.yaml              # Robot parameters
├── worlds/                    # Gazebo world files
│   └── test_world.sdf                 # Test environment with obstacles
├── models/robot/              # Robot model definition
│   ├── model.sdf                      # Robot SDF description
│   └── model.config                   # Model metadata
└── scripts/                   # Python executables
    ├── teleop_keyboard.py             # Teleoperation node
    └── robot_monitor.py               # System monitoring node
```

## Requirements

### System Requirements
- **OS**: Ubuntu 20.04/22.04 (tested)
- **ROS 2**: Humble/Iron/Jazzy/Rolling
- **Gazebo**: Garden/Harmonic (compatible with ROS 2 version)
- **Python**: 3.8+

### ROS 2 Dependencies
```bash
# Core ROS 2 packages
ros-${ROS_DISTRO}-rclpy
ros-${ROS_DISTRO}-geometry-msgs
ros-${ROS_DISTRO}-sensor-msgs
ros-${ROS_DISTRO}-nav-msgs
ros-${ROS_DISTRO}-std-msgs
ros-${ROS_DISTRO}-tf2-ros

# Gazebo integration
ros-${ROS_DISTRO}-ros-gz-sim
ros-${ROS_DISTRO}-ros-gz-bridge
ros-${ROS_DISTRO}-ros-gz-interfaces

# Robot state and visualization
ros-${ROS_DISTRO}-robot-state-publisher
ros-${ROS_DISTRO}-joint-state-publisher
ros-${ROS_DISTRO}-rviz2

# Diagnostics
ros-${ROS_DISTRO}-diagnostic-msgs
ros-${ROS_DISTRO}-diagnostic-updater
```

### Python Dependencies
```bash
pip install pynput termios
```

## Installation

### 1. Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone/Copy Package
```bash
# Copy your robot_simulation package to src/
cp -r /path/to/robot_simulation .
```

### 3. Install Dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select robot_simulation
```

### 5. Source Environment
```bash
source ~/ros2_ws/install/setup.bash
# Add to ~/.bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Usage

### Quick Start
```bash
# Launch complete simulation
ros2 launch robot_simulation robot_simulation.launch.py

# Launch with teleoperation
ros2 launch robot_simulation robot_simulation.launch.py teleop:=true

# Launch with RViz visualization
ros2 launch robot_simulation robot_simulation.launch.py rviz:=true

# Headless mode (no GUI)
ros2 launch robot_simulation robot_simulation.launch.py gui:=false
```

### Advanced Usage

#### Custom Robot Position
```bash
ros2 launch robot_simulation robot_simulation.launch.py \
    robot_x:=2.0 robot_y:=1.0 robot_z:=0.5 robot_yaw:=0.0
```

#### Bridge Only Mode
```bash
# Start Gazebo manually, then launch bridge
gz sim ~/ros2_ws/src/robot_simulation/worlds/test_world.sdf
ros2 launch robot_simulation bridge_only.launch.py
```

#### Spawn Additional Robot
```bash
ros2 launch robot_simulation robot_spawn.launch.py \
    robot_name:=robot2 robot_x:=5.0 robot_y:=5.0
```

### Teleoperation Controls

When teleoperation is enabled:

```
Movement Controls:
------------------
W/↑     : Move forward
S/↓     : Move backward  
A/←     : Strafe left
D/→     : Strafe right
Q       : Rotate left
E       : Rotate right

Speed Controls:
---------------
Z       : Increase linear speed
X       : Decrease linear speed
C       : Increase angular speed
V       : Decrease angular speed
R       : Reset to default speeds

Safety Controls:
----------------
SPACE   : Emergency stop
ESC     : Quit
Ctrl+C  : Force quit
```

## Configuration

### Bridge Configuration (`config/bridge_config.yaml`)

The bridge configuration maps Gazebo topics to ROS 2 topics:

```yaml
# Example bridge configuration
- ros_topic_name: "/lidar_front/scan"
  gz_topic_name: "/lidar_front"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

### Robot Parameters (`config/robot_params.yaml`)

Contains robot-specific settings:

```yaml
robot_state_publisher:
  ros__parameters:
    use_sim_time: true
    publish_frequency: 50.0
    
# Sensor parameters, control limits, etc.
```

## Nodes

### robot_monitor
**Purpose**: System health monitoring and diagnostics
**Subscriptions**: All sensor topics, odometry, commands
**Publications**: `/diagnostics`, `/emergency_stop`

**Features**:
- Topic rate monitoring
- Sensor data validation
- Safety threshold checking
- Emergency stop detection

### teleop_keyboard  
**Purpose**: Keyboard-based robot control
**Publications**: `/cmd_vel`, `/emergency_stop`, `/diagnostics`

**Features**:
- WASD/Arrow key control
- Speed adjustment
- Emergency stop
- Safety timeouts

## Topics

### Sensor Topics
- `/lidar_front/scan` - Front LiDAR data
- `/lidar_back/scan` - Rear LiDAR data  
- `/camera/image_raw` - Camera images
- `/camera/camera_info` - Camera calibration

### Control Topics
- `/cmd_vel` - Velocity commands
- `/odom` - Robot odometry
- `/joint_states` - Joint positions

### Diagnostic Topics
- `/diagnostics` - System diagnostics
- `/emergency_stop` - Emergency stop status

### Transform Topics
- `/tf` - Dynamic transforms
- `/tf_static` - Static transforms

## Launch Files

### robot_simulation.launch.py
**Purpose**: Complete simulation startup
**Arguments**:
- `world`: Path to world file
- `gui`: Enable/disable Gazebo GUI
- `rviz`: Launch RViz
- `teleop`: Enable teleoperation
- `robot_x/y/z/yaw`: Initial robot pose

### bridge_only.launch.py  
**Purpose**: ROS-Gazebo bridge only
**Arguments**:
- `bridge_config`: Bridge configuration file
- `verbose`: Verbose output

### robot_spawn.launch.py
**Purpose**: Spawn robot in existing simulation
**Arguments**:
- `robot_name`: Robot instance name
- `model_path`: Path to robot model
- `robot_x/y/z/roll/pitch/yaw`: Spawn pose

## Troubleshooting

### Common Issues

#### Gazebo Won't Start
```bash
# Check Gazebo installation
gz sim --version

# Set environment variables
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/robot_simulation
```

#### Bridge Connection Issues
```bash
# Check bridge status
ros2 topic list | grep gz

# Verify bridge configuration
ros2 param get /ros_gz_bridge config_file
```

#### No Sensor Data
```bash
# Check Gazebo topics
gz topic -l

# Check ROS topics
ros2 topic list

# Monitor bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args --log-level DEBUG
```

#### Robot Won't Move
```bash
# Check velocity commands
ros2 topic echo /cmd_vel

# Check emergency stop
ros2 topic echo /emergency_stop

# Manual velocity test
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.1}"
```

### Debugging Commands

```bash
# Check node status
ros2 node list
ros2 node info /robot_monitor

# Topic information
ros2 topic info /lidar_front/scan
ros2 topic hz /odom

# Parameter inspection
ros2 param list /robot_state_publisher
ros2 param get /robot_monitor expected_rates

# Transform debugging
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link lidar_front_link
```

### Log Analysis
```bash
# ROS 2 logs
ros2 doctor
ros2 wtf

# Node-specific logs
ros2 run robot_simulation robot_monitor --ros-args --log-level DEBUG
```

## Performance Optimization

### Gazebo Performance
- Reduce physics step size for better performance
- Disable GUI for headless operation
- Adjust sensor update rates based on requirements

### Bridge Optimization
- Use appropriate QoS settings
- Minimize bridged topics
- Adjust queue sizes based on data rates

## Contributing

### Development Setup
1. Fork the repository
2. Create feature branch
3. Follow ROS 2 coding standards
4. Add tests for new features
5. Submit pull request

### Code Style
- Follow PEP 8 for Python code
- Use ROS 2 naming conventions
- Document all public functions
- Include type hints where appropriate

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Search existing issues
3. Create a new issue with:
   - System information
   - Steps to reproduce
   - Expected vs actual behavior
   - Relevant log outputs

## Acknowledgments

- ROS 2 community for the excellent documentation
- Gazebo team for the simulation platform
- Contributors and testers