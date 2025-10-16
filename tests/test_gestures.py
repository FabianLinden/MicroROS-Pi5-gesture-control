"""
Test gesture recognition without robot control
"""

import cv2
import yaml
import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from gesture_control.gesture_recognizer import GestureRecognizer
from utils.visualization import GestureVisualizer


def test_gestures(config_path, camera_index=0):
    """Test gesture recognition system"""
    
    print("Loading configuration...")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    print("Initializing gesture recognizer...")
    recognizer = GestureRecognizer(config)
    visualizer = GestureVisualizer(config)
    
    print("Opening camera...")
    camera = cv2.VideoCapture(camera_index)
    
    if not camera.isOpened():
        print("ERROR: Cannot open camera")
        return
    
    print("\nGesture Recognition Test")
    print("=" * 50)
    print("Try different hand gestures in front of the camera")
    print("Press 'q' to quit\n")
    
    try:
        while True:
            ret, frame = camera.read()
            
            if not ret:
                print("ERROR: Failed to capture frame")
                break
            
            # Process frame
            command, params, debug_info = recognizer.process_frame(frame)
            
            # Print detected command
            if command:
                print(f"Command: {command:30s} | Mode: {params.get('mode', 'N/A'):20s} | Speed: {params.get('speed_level', 'N/A')}")
            
            # Visualize
            vis_frame = visualizer.draw(frame, debug_info)
            cv2.imshow('Gesture Recognition Test', vis_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    finally:
        print("\nCleaning up...")
        camera.release()
        cv2.destroyAllWindows()
        recognizer.close()
        print("Test completed!")


if __name__ == '__main__':
    # Default config path
    config_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config',
        'gesture_config.yaml'
    )
    
    camera_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    
    test_gestures(config_path, camera_idx)

