"""
Main entry point for MicroROS-Pi5 Gesture Control System
"""

import argparse
import sys
import os
from pathlib import Path

# Add current directory to path
sys.path.append(str(Path(__file__).parent))


def main():
    """Main entry point with argument parsing"""
    
    parser = argparse.ArgumentParser(
        description='MicroROS-Pi5 Gesture Control System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with default settings
  python main.py
  
  # Test camera
  python main.py --test-camera
  
  # Test gestures without robot control
  python main.py --test-gestures
  
  # Use custom config file
  python main.py --config my_config.yaml
  
  # Use different camera
  python main.py --camera 1
        """
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default='config/gesture_config.yaml',
        help='Path to configuration file (default: config/gesture_config.yaml)'
    )
    
    parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Camera index (default: 0)'
    )
    
    parser.add_argument(
        '--test-camera',
        action='store_true',
        help='Test camera functionality only'
    )
    
    parser.add_argument(
        '--test-gestures',
        action='store_true',
        help='Test gesture recognition without robot control'
    )
    
    parser.add_argument(
        '--no-viz',
        action='store_true',
        help='Disable visualization window'
    )
    
    args = parser.parse_args()
    
    # Test camera
    if args.test_camera:
        from tests.test_camera import test_camera
        test_camera(args.camera)
        return
    
    # Test gestures
    if args.test_gestures:
        from tests.test_gestures import test_gestures
        test_gestures(args.config, args.camera)
        return
    
    # Run full ROS2 node
    print("Starting MicroROS-Pi5 Gesture Control System...")
    print(f"Config: {args.config}")
    print(f"Camera: {args.camera}")
    
    # Validate configuration
    from utils.config_validator import validate_config_file
    
    print("\nValidating configuration...")
    is_valid, errors, config = validate_config_file(args.config)
    
    if not is_valid:
        print("✗ Configuration validation failed:")
        for error in errors:
            print(f"  - {error}")
        print("\nPlease fix the configuration errors and try again.")
        return
    
    print("✓ Configuration is valid")
    
    # Update config if no-viz specified
    if args.no_viz:
        import yaml
        config['show_visualization'] = False
        with open(args.config, 'w') as f:
            yaml.dump(config, f)
    
    # Import and run ROS2 node
    from ros2_nodes.gesture_control_node import main as node_main
    node_main()


if __name__ == '__main__':
    main()
