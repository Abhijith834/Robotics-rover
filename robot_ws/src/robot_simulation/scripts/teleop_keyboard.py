#!/usr/bin/env python3

"""
Enhanced Keyboard Teleoperation Node for Robot Simulation

This node provides keyboard control for the robot with safety features,
speed adjustment, and emergency stop functionality.

Controls:
    WASD / Arrow keys: Move robot
    Q/E: Rotate left/right  
    Z/X: Increase/decrease linear speed
    C/V: Increase/decrease angular speed
    Space: Emergency stop
    R: Reset speeds to default
    ESC/Ctrl+C: Quit

Author: Your Name
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import sys
import select
import termios
import tty
import threading
import time

class TeleopKeyboard(Node):
    """Enhanced keyboard teleoperation node with safety features."""
    
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed_step', 0.1)
        self.declare_parameter('angular_speed_step', 0.1)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('key_timeout', 0.1)
        self.declare_parameter('repeat_rate', 10.0)
        self.declare_parameter('emergency_stop_enabled', True)
        self.declare_parameter('safety_timeout', 0.5)
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_step = self.get_parameter('linear_speed_step').value
        self.angular_step = self.get_parameter('angular_speed_step').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.key_timeout = self.get_parameter('key_timeout').value
        self.repeat_rate = self.get_parameter('repeat_rate').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
        self.safety_timeout = self.get_parameter('safety_timeout').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Robot state
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.emergency_stop = False
        self.last_key_time = time.time()
        
        # Control settings
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_mapping = {
            'w': (1, 0),    # forward
            's': (-1, 0),   # backward
            'a': (0, 1),    # left
            'd': (0, -1),   # right
            'q': (0, 1),    # rotate left
            'e': (0, -1),   # rotate right
            # Arrow keys
            '\x1b[A': (1, 0),   # Up arrow
            '\x1b[B': (-1, 0),  # Down arrow
            '\x1b[D': (0, 1),   # Left arrow
            '\x1b[C': (0, -1),  # Right arrow
        }
        
        # Create timers
        self.publish_timer = self.create_timer(1.0/self.repeat_rate, self.publish_velocity)
        self.diagnostics_timer = self.create_timer(1.0, self.publish_diagnostics)
        self.safety_timer = self.create_timer(0.1, self.check_safety_timeout)
        
        # Start keyboard thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info("Teleop Keyboard node started")
        self.print_usage()
    
    def print_usage(self):
        """Print usage instructions."""
        usage = """
        Enhanced Robot Teleoperation
        ============================
        
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
        
        Current speeds: Linear={:.2f}, Angular={:.2f}
        Max speeds: Linear={:.2f}, Angular={:.2f}
        """.format(self.target_linear, self.target_angular, 
                  self.max_linear, self.max_angular)
        
        print(usage)
    
    def get_key(self):
        """Get a key press from the keyboard."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        
        # Handle escape sequences (arrow keys)
        if key == '\x1b':
            key += sys.stdin.read(2)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def keyboard_loop(self):
        """Main keyboard input loop."""
        while rclpy.ok():
            try:
                if select.select([sys.stdin], [], [], self.key_timeout) == ([sys.stdin], [], []):
                    key = self.get_key()
                    self.process_key(key)
                    self.last_key_time = time.time()
            except Exception as e:
                self.get_logger().error(f"Keyboard error: {e}")
                break
    
    def process_key(self, key):
        """Process a key press."""
        key_lower = key.lower()
        
        # Emergency stop
        if key == ' ':
            self.emergency_stop = True
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.get_logger().warn("EMERGENCY STOP ACTIVATED!")
            return
        
        # Quit
        if key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
            self.get_logger().info("Shutting down teleop...")
            rclpy.shutdown()
            return
        
        # Reset emergency stop
        if key_lower == 'r':
            self.emergency_stop = False
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.get_logger().info("Speeds reset to default")
            return
        
        # Speed adjustments
        if key_lower == 'z':
            self.max_linear = min(self.max_linear + self.linear_step, 2.0)
            self.get_logger().info(f"Max linear speed: {self.max_linear:.2f} m/s")
            return
        elif key_lower == 'x':
            self.max_linear = max(self.max_linear - self.linear_step, 0.1)
            self.get_logger().info(f"Max linear speed: {self.max_linear:.2f} m/s")
            return
        elif key_lower == 'c':
            self.max_angular = min(self.max_angular + self.angular_step, 2.0)
            self.get_logger().info(f"Max angular speed: {self.max_angular:.2f} rad/s")
            return
        elif key_lower == 'v':
            self.max_angular = max(self.max_angular - self.angular_step, 0.1)
            self.get_logger().info(f"Max angular speed: {self.max_angular:.2f} rad/s")
            return
        
        # Movement commands
        if key in self.key_mapping:
            linear_dir, angular_dir = self.key_mapping[key]
            self.target_linear = linear_dir * self.max_linear
            self.target_angular = angular_dir * self.max_angular
            self.emergency_stop = False
        elif key_lower in self.key_mapping:
            linear_dir, angular_dir = self.key_mapping[key_lower]
            self.target_linear = linear_dir * self.max_linear
            self.target_angular = angular_dir * self.max_angular
            self.emergency_stop = False
        else:
            # Unknown key - stop movement
            self.target_linear = 0.0
            self.target_angular = 0.0
    
    def check_safety_timeout(self):
        """Check for safety timeout - stop robot if no recent key presses."""
        if time.time() - self.last_key_time > self.safety_timeout:
            self.target_linear = 0.0
            self.target_angular = 0.0
    
    def publish_velocity(self):
        """Publish velocity commands with smoothing."""
        # Smooth acceleration/deceleration
        alpha = 0.3  # Smoothing factor
        self.current_linear = alpha * self.target_linear + (1 - alpha) * self.current_linear
        self.current_angular = alpha * self.target_angular + (1 - alpha) * self.current_angular
        
        # Create and publish Twist message
        twist = Twist()
        
        if not self.emergency_stop:
            twist.linear.x = self.current_linear
            twist.angular.z = self.current_angular
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        
        # Publish emergency stop status
        if self.emergency_stop_enabled:
            stop_msg = Bool()
            stop_msg.data = self.emergency_stop
            self.emergency_stop_pub.publish(stop_msg)
    
    def publish_diagnostics(self):
        """Publish diagnostic information."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Teleop status
        teleop_status = DiagnosticStatus()
        teleop_status.name = "teleop_keyboard"
        teleop_status.level = DiagnosticStatus.OK
        teleop_status.message = "Teleoperation active"
        
        if self.emergency_stop:
            teleop_status.level = DiagnosticStatus.WARN
            teleop_status.message = "Emergency stop active"
        
        teleop_status.values = [
            KeyValue(key="current_linear_speed", value=str(self.current_linear)),
            KeyValue(key="current_angular_speed", value=str(self.current_angular)),
            KeyValue(key="max_linear_speed", value=str(self.max_linear)),
            KeyValue(key="max_angular_speed", value=str(self.max_angular)),
            KeyValue(key="emergency_stop", value=str(self.emergency_stop)),
            KeyValue(key="last_key_time", value=str(time.time() - self.last_key_time))
        ]
        
        diag_array.status.append(teleop_status)
        self.diagnostics_pub.publish(diag_array)
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        teleop_node = TeleopKeyboard()
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Restore terminal
        if 'teleop_node' in locals():
            teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()