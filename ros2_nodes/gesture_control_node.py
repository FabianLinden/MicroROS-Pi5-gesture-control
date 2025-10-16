"""
ROS2 Node for Gesture-Based Robot Control
Compatible with MicroROS-Pi5 ESP32 control board
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
import cv2
import yaml
import sys
import os
import logging
from pathlib import Path

# Add parent directory to path to import gesture_control
sys.path.append(str(Path(__file__).parent.parent))

from gesture_control.gesture_recognizer import GestureRecognizer
from utils.visualization import GestureVisualizer


class GestureControlNode(Node):
    """ROS2 node for gesture-based robot control"""
    
    def __init__(self, config_path: str, camera_index: int = 0):
        super().__init__('gesture_control_node')
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Setup logging
        self._setup_logging()
        
        # Initialize gesture recognizer
        self.recognizer = GestureRecognizer(self.config)
        
        # Initialize visualizer
        self.visualizer = GestureVisualizer(self.config)
        
        # Initialize camera
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 
                       self.config.get('camera_width', 640))
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 
                       self.config.get('camera_height', 480))
        
        if not self.camera.isOpened():
            self.get_logger().error("Failed to open camera!")
            raise RuntimeError("Camera not accessible")
        
        # ROS2 Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.config['ros_topics']['cmd_vel'],
            10
        )
        
        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            self.config['ros_topics']['servo_control'],
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            self.config['ros_topics']['gesture_status'],
            10
        )
        
        # Safety timer - stop robot if no gesture detected
        self.last_command_time = self.get_clock().now()
        self.safety_timeout = self.config.get('safety_timeout', 2.0)  # seconds
        self.create_timer(0.1, self.safety_check_callback)
        
        # Main processing timer
        self.create_timer(1.0 / self.config.get('fps', 30), self.process_frame_callback)
        
        # Current robot state
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.gimbal_pan = 90.0  # Center position
        self.gimbal_tilt = 90.0  # Center position
        
        self.get_logger().info("Gesture Control Node initialized")
        self.get_logger().info(f"Control mode: {self.recognizer.gesture_mapper.get_current_mode()}")
        self.get_logger().info(f"Log level: {self.config.get('log_level', 'INFO')}")
    
    def _setup_logging(self):
        """Setup logging configuration"""
        log_level_str = self.config.get('log_level', 'INFO').upper()
        
        # Map string to logging level
        log_levels = {
            'DEBUG': logging.DEBUG,
            'INFO': logging.INFO,
            'WARN': logging.WARN,
            'WARNING': logging.WARNING,
            'ERROR': logging.ERROR,
            'CRITICAL': logging.CRITICAL
        }
        
        log_level = log_levels.get(log_level_str, logging.INFO)
        
        # Configure Python logging
        logging.basicConfig(
            level=log_level,
            format='[%(levelname)s] [%(name)s]: %(message)s'
        )
        
        self.logger = logging.getLogger('gesture_control')
    
    def process_frame_callback(self):
        """Main processing loop - capture frame and process gestures"""
        
        ret, frame = self.camera.read()
        
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
        
        # Process frame for gestures
        command, params, debug_info = self.recognizer.process_frame(frame)
        
        # Execute command if detected
        if command:
            # Log command with confidence scores
            static_conf = debug_info.get('static_confidence', 0.0)
            dynamic_conf = debug_info.get('dynamic_confidence', 0.0)
            
            self.logger.debug(
                f"Command: {command} | Static: {static_conf:.2f} | Dynamic: {dynamic_conf:.2f}"
            )
            
            self.execute_command(command, params)
            self.last_command_time = self.get_clock().now()
            
            # Publish status
            status_msg = String()
            status_msg.data = f"{command}:{params.get('mode', 'unknown')}"
            self.status_pub.publish(status_msg)
        
        # Visualize (if enabled)
        if self.config.get('show_visualization', True):
            vis_frame = self.visualizer.draw(frame, debug_info)
            cv2.imshow('Gesture Control', vis_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Quit signal received")
                self.cleanup()
                rclpy.shutdown()
    
    def execute_command(self, command: str, params: dict):
        """Execute robot command based on gesture"""
        
        speed_mult = params.get('speed_multiplier', 0.6)
        max_linear = self.config['robot_limits']['max_linear_speed']
        max_angular = self.config['robot_limits']['max_angular_speed']
        
        # Create Twist message for movement commands
        twist = Twist()
        
        # Movement commands
        if command == "move_forward" or command == "move_forward_slow":
            twist.linear.x = max_linear * speed_mult
            self.current_linear_vel = twist.linear.x
            
        elif command == "move_backward" or command == "move_backward_slow":
            twist.linear.x = -max_linear * speed_mult
            self.current_linear_vel = twist.linear.x
            
        elif command == "turn_left" or command == "turn_left_slow":
            twist.angular.z = max_angular * speed_mult
            self.current_angular_vel = twist.angular.z
            
        elif command == "turn_right" or command == "turn_right_slow":
            twist.angular.z = -max_angular * speed_mult
            self.current_angular_vel = twist.angular.z
            
        elif command == "rotate_clockwise":
            twist.angular.z = -max_angular * speed_mult * 0.7
            self.current_angular_vel = twist.angular.z
            
        elif command == "rotate_counter_clockwise":
            twist.angular.z = max_angular * speed_mult * 0.7
            self.current_angular_vel = twist.angular.z
            
        elif command == "emergency_stop" or command == "hold_position":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.get_logger().warn("EMERGENCY STOP!")
            self.logger.warning("EMERGENCY STOP activated!")
        
        # Publish movement command
        if command.startswith("move_") or command.startswith("turn_") or \
           command.startswith("rotate_") or command in ["emergency_stop", "hold_position"]:
            self.cmd_vel_pub.publish(twist)
        
        # Gimbal control commands
        elif command.startswith("gimbal_"):
            self.execute_gimbal_command(command, speed_mult)
        
        # Speed adjustment commands
        elif command.startswith("set_speed_") or command in ["increase_speed", "decrease_speed"]:
            new_speed = params.get('new_speed', self.recognizer.gesture_mapper.get_speed_level())
            self.get_logger().info(f"Speed level: {new_speed}")
            self.logger.info(f"Speed adjusted to level {new_speed}")
        
        # Mode switch commands
        elif command.startswith("mode_switch_"):
            mode = command.replace("mode_switch_", "")
            self.get_logger().info(f"Switched to {mode} mode")
            self.logger.info(f"Control mode changed to: {mode}")
    
    def execute_gimbal_command(self, command: str, speed_mult: float):
        """Execute gimbal control command"""
        
        gimbal_step = self.config['robot_limits']['gimbal_step'] * speed_mult
        
        if command == "gimbal_pan_left":
            self.gimbal_pan = max(0.0, self.gimbal_pan - gimbal_step)
        
        elif command == "gimbal_pan_right":
            self.gimbal_pan = min(180.0, self.gimbal_pan + gimbal_step)
        
        elif command == "gimbal_tilt_up":
            self.gimbal_tilt = max(0.0, self.gimbal_tilt - gimbal_step)
        
        elif command == "gimbal_tilt_down":
            self.gimbal_tilt = min(180.0, self.gimbal_tilt + gimbal_step)
        
        elif command == "gimbal_center":
            self.gimbal_pan = 90.0
            self.gimbal_tilt = 90.0
        
        # Publish servo positions [pan, tilt]
        servo_msg = Float32MultiArray()
        servo_msg.data = [float(self.gimbal_pan), float(self.gimbal_tilt)]
        self.servo_pub.publish(servo_msg)
    
    def safety_check_callback(self):
        """Safety check - stop robot if no command received recently"""
        
        time_since_last = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        
        if time_since_last > self.safety_timeout:
            # No command for too long - stop robot
            if self.current_linear_vel != 0.0 or self.current_angular_vel != 0.0:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                self.current_linear_vel = 0.0
                self.current_angular_vel = 0.0
                
                self.get_logger().warn("Safety timeout - stopping robot")
                self.logger.warning(f"Safety timeout ({self.safety_timeout}s) - robot stopped")
    
    def cleanup(self):
        """Cleanup resources"""
        self.get_logger().info("Cleaning up...")
        
        # Stop robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Release camera
        self.camera.release()
        cv2.destroyAllWindows()
        
        # Close recognizer
        self.recognizer.close()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    # Default config path
    config_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config',
        'gesture_config.yaml'
    )
    
    try:
        node = GestureControlNode(config_path)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

