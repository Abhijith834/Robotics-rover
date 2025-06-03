#!/usr/bin/env python3

"""
Robot Monitor Node for System Health and Safety

This node monitors the robot's sensor data, system health, and safety parameters.
It publishes diagnostic information and alerts for anomalies.

Features:
- Topic rate monitoring
- Sensor data validation
- Safety threshold checking  
- System health diagnostics
- Emergency stop detection
- Performance metrics

Author: Your Name
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time
import threading
import math
from collections import defaultdict, deque

class RobotMonitor(Node):
    """Comprehensive robot monitoring and diagnostics node."""
    
    def __init__(self):
        super().__init__('robot_monitor')
        
        # Declare parameters
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('monitor_topics', [
            '/lidar_front/scan', '/lidar_back/scan', '/camera/image_raw',
            '/odom', '/cmd_vel', '/joint_states'
        ])
        self.declare_parameter('expected_rates', {
            '/lidar_front/scan': 10.0,
            '/lidar_back/scan': 10.0, 
            '/camera/image_raw': 30.0,
            '/odom': 50.0,
            '/joint_states': 30.0
        })
        self.declare_parameter('min_distance_threshold', 0.3)
        self.declare_parameter('max_linear_velocity', 2.0)
        self.declare_parameter('max_angular_velocity', 1.5)
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.monitor_topics = self.get_parameter('monitor_topics').value
        self.expected_rates = self.get_parameter('expected_rates').value
        self.min_distance = self.get_parameter('min_distance_threshold').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.publish_diag = self.get_parameter('publish_diagnostics').value
        self.diag_topic = self.get_parameter('diagnostics_topic').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.lidar_front_sub = self.create_subscription(
            LaserScan, '/lidar_front/scan', self.lidar_front_callback, sensor_qos)
        self.lidar_back_sub = self.create_subscription(
            LaserScan, '/lidar_back/scan', self.lidar_back_callback, sensor_qos)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, reliable_qos)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, reliable_qos)
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, reliable_qos)
        
        # Publishers
        if self.publish_diag:
            self.diagnostics_pub = self.create_publisher(
                DiagnosticArray, self.diag_topic, 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Monitoring data structures
        self.topic_stats = defaultdict(lambda: {
            'last_msg_time': 0.0,
            'msg_count': 0,
            'rate_history': deque(maxlen=10),
            'last_rate_calc': 0.0
        })
        
        # Sensor data storage
        self.latest_lidar_front = None
        self.latest_lidar_back = None
        self.latest_odom = None
        self.latest_cmd_vel = None
        self.latest_joints = None
        
        # Safety flags
        self.emergency_stop_triggered = False
        self.safety_alerts = []
        
        # Statistics
        self.start_time = time.time()
        self.total_warnings = 0
        self.total_errors = 0
        
        # Create timers
        self.monitor_timer = self.create_timer(
            1.0 / self.update_rate, self.monitor_callback)
        self.diagnostics_timer = self.create_timer(1.0, self.publish_diagnostics)
        
        self.get_logger().info("Robot Monitor node started")
        self.get_logger().info(f"Monitoring topics: {self.monitor_topics}")
    
    def lidar_front_callback(self, msg):
        """Handle front LiDAR data."""
        self.update_topic_stats('/lidar_front/scan')
        self.latest_lidar_front = msg
        self.check_lidar_safety(msg, 'front')
    
    def lidar_back_callback(self, msg):
        """Handle back LiDAR data."""
        self.update_topic_stats('/lidar_back/scan')
        self.latest_lidar_back = msg
        self.check_lidar_safety(msg, 'back')
    
    def camera_callback(self, msg):
        """Handle camera data."""
        self.update_topic_stats('/camera/image_raw')
        # Could add image analysis here
    
    def odom_callback(self, msg):
        """Handle odometry data."""
        self.update_topic_stats('/odom')
        self.latest_odom = msg
        self.check_velocity_limits(msg)
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands."""
        self.update_topic_stats('/cmd_vel')
        self.latest_cmd_vel = msg
        self.check_command_safety(msg)
    
    def joint_states_callback(self, msg):
        """Handle joint state data."""
        self.update_topic_stats('/joint_states')
        self.latest_joints = msg
    
    def update_topic_stats(self, topic_name):
        """Update statistics for a topic."""
        current_time = time.time()
        stats = self.topic_stats[topic_name]
        
        # Update message count and time
        stats['msg_count'] += 1
        last_time = stats['last_msg_time']
        stats['last_msg_time'] = current_time
        
        # Calculate rate if we have a previous message
        if last_time > 0:
            dt = current_time - last_time
            if dt > 0:
                rate = 1.0 / dt
                stats['rate_history'].append(rate)
    
    def check_lidar_safety(self, scan_msg, sensor_name):
        """Check LiDAR data for safety violations."""
        min_range = min([r for r in scan_msg.ranges if not math.isinf(r) and not math.isnan(r)], default=float('inf'))
        
        if min_range < self.min_distance:
            alert = f"Obstacle detected at {min_range:.2f}m by {sensor_name} LiDAR (threshold: {self.min_distance}m)"
            if alert not in self.safety_alerts:
                self.safety_alerts.append(alert)
                self.get_logger().warn(alert)
                self.trigger_emergency_stop("Obstacle too close")
    
    def check_velocity_limits(self, odom_msg):
        """Check if robot velocity exceeds safety limits."""
        linear_vel = math.sqrt(
            odom_msg.twist.twist.linear.x**2 + 
            odom_msg.twist.twist.linear.y**2
        )
        angular_vel = abs(odom_msg.twist.twist.angular.z)
        
        if linear_vel > self.max_linear_vel:
            alert = f"Linear velocity {linear_vel:.2f} exceeds limit {self.max_linear_vel}"
            if alert not in self.safety_alerts:
                self.safety_alerts.append(alert)
                self.get_logger().warn(alert)
                self.total_warnings += 1
        
        if angular_vel > self.max_angular_vel:
            alert = f"Angular velocity {angular_vel:.2f} exceeds limit {self.max_angular_vel}"
            if alert not in self.safety_alerts:
                self.safety_alerts.append(alert)
                self.get_logger().warn(alert)
                self.total_warnings += 1
    
    def check_command_safety(self, cmd_msg):
        """Check velocity commands for safety."""
        linear_cmd = abs(cmd_msg.linear.x)
        angular_cmd = abs(cmd_msg.angular.z)
        
        if linear_cmd > self.max_linear_vel * 1.1:  # 10% tolerance
            self.get_logger().warn(f"Command linear velocity {linear_cmd:.2f} exceeds safe limit")
        
        if angular_cmd > self.max_angular_vel * 1.1:  # 10% tolerance  
            self.get_logger().warn(f"Command angular velocity {angular_cmd:.2f} exceeds safe limit")
    
    def trigger_emergency_stop(self, reason):
        """Trigger emergency stop."""
        if not self.emergency_stop_triggered:
            self.emergency_stop_triggered = True
            self.get_logger().error(f"EMERGENCY STOP: {reason}")
            
            # Publish emergency stop
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)
            self.total_errors += 1
    
    def monitor_callback(self):
        """Main monitoring callback."""
        current_time = time.time()
        
        # Check topic rates
        for topic in self.monitor_topics:
            stats = self.topic_stats[topic]
            expected_rate = self.expected_rates.get(topic, 1.0)
            
            # Calculate average rate
            if len(stats['rate_history']) > 0:
                avg_rate = sum(stats['rate_history']) / len(stats['rate_history'])
            else:
                avg_rate = 0.0
            
            # Check if topic is active
            time_since_last = current_time - stats['last_msg_time']
            if time_since_last > 2.0 / expected_rate:  # Allow 2x expected period
                alert = f"Topic {topic} inactive for {time_since_last:.1f}s"
                if alert not in self.safety_alerts:
                    self.safety_alerts.append(alert)
                    self.get_logger().warn(alert)
                    self.total_warnings += 1
            
            # Check rate
            if avg_rate < expected_rate * 0.5:  # 50% of expected rate
                alert = f"Topic {topic} rate {avg_rate:.1f}Hz below expected {expected_rate}Hz"
                if alert not in self.safety_alerts:
                    self.safety_alerts.append(alert)
                    self.get_logger().warn(alert)
                    self.total_warnings += 1
        
        # Clear old alerts (older than 5 seconds)
        self.safety_alerts = self.safety_alerts[-10:]  # Keep last 10 alerts
    
    def publish_diagnostics(self):
        """Publish diagnostic information."""
        if not self.publish_diag:
            return
            
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Overall system status
        system_status = DiagnosticStatus()
        system_status.name = "robot_system"
        
        if self.emergency_stop_triggered:
            system_status.level = DiagnosticStatus.ERROR
            system_status.message = "Emergency stop active"
        elif len(self.safety_alerts) > 0:
            system_status.level = DiagnosticStatus.WARN
            system_status.message = f"{len(self.safety_alerts)} active alerts"
        else:
            system_status.level = DiagnosticStatus.OK
            system_status.message = "System operating normally"
        
        # System values
        uptime = time.time() - self.start_time
        system_status.values = [
            KeyValue(key="uptime", value=f"{uptime:.1f}"),
            KeyValue(key="total_warnings", value=str(self.total_warnings)),
            KeyValue(key="total_errors", value=str(self.total_errors)),
            KeyValue(key="active_alerts", value=str(len(self.safety_alerts))),
            KeyValue(key="emergency_stop", value=str(self.emergency_stop_triggered))
        ]
        
        diag_array.status.append(system_status)
        
        # Topic statistics
        for topic, stats in self.topic_stats.items():
            topic_status = DiagnosticStatus()
            topic_status.name = f"topic_{topic.replace('/', '_')}"
            
            # Calculate rate
            if len(stats['rate_history']) > 0:
                avg_rate = sum(stats['rate_history']) / len(stats['rate_history'])
                expected = self.expected_rates.get(topic, 1.0)
                
                if avg_rate < expected * 0.5:
                    topic_status.level = DiagnosticStatus.WARN
                    topic_status.message = f"Low rate: {avg_rate:.1f}Hz"
                else:
                    topic_status.level = DiagnosticStatus.OK
                    topic_status.message = f"Rate: {avg_rate:.1f}Hz"
            else:
                topic_status.level = DiagnosticStatus.ERROR
                topic_status.message = "No messages received"
                avg_rate = 0.0
            
            topic_status.values = [
                KeyValue(key="message_count", value=str(stats['msg_count'])),
                KeyValue(key="average_rate", value=f"{avg_rate:.2f}"),
                KeyValue(key="expected_rate", value=str(self.expected_rates.get(topic, 1.0))),
                KeyValue(key="last_message_age", value=f"{time.time() - stats['last_msg_time']:.2f}")
            ]
            
            diag_array.status.append(topic_status)
        
        # Sensor-specific diagnostics
        if self.latest_lidar_front:
            self.add_lidar_diagnostics(diag_array, self.latest_lidar_front, "front_lidar")
        
        if self.latest_lidar_back:
            self.add_lidar_diagnostics(diag_array, self.latest_lidar_back, "back_lidar")
        
        self.diagnostics_pub.publish(diag_array)
    
    def add_lidar_diagnostics(self, diag_array, scan_msg, name):
        """Add LiDAR-specific diagnostics."""
        lidar_status = DiagnosticStatus()
        lidar_status.name = name
        
        # Calculate statistics
        valid_ranges = [r for r in scan_msg.ranges if not math.isinf(r) and not math.isnan(r)]
        
        if len(valid_ranges) > 0:
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            avg_range = sum(valid_ranges) / len(valid_ranges)
            
            if min_range < self.min_distance:
                lidar_status.level = DiagnosticStatus.WARN
                lidar_status.message = f"Obstacle at {min_range:.2f}m"
            else:
                lidar_status.level = DiagnosticStatus.OK
                lidar_status.message = "Normal operation"
        else:
            lidar_status.level = DiagnosticStatus.ERROR
            lidar_status.message = "No valid range data"
            min_range = max_range = avg_range = 0.0
        
        lidar_status.values = [
            KeyValue(key="min_range", value=f"{min_range:.2f}"),
            KeyValue(key="max_range", value=f"{max_range:.2f}"),
            KeyValue(key="avg_range", value=f"{avg_range:.2f}"),
            KeyValue(key="valid_points", value=str(len(valid_ranges))),
            KeyValue(key="total_points", value=str(len(scan_msg.ranges)))
        ]
        
        diag_array.status.append(lidar_status)

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        monitor_node = RobotMonitor()
        rclpy.spin(monitor_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()