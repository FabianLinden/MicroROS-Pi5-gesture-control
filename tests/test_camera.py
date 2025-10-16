"""
Test camera functionality
"""

import cv2
import sys


def test_camera(camera_index=0):
    """Test if camera is accessible and working"""
    
    print(f"Testing camera {camera_index}...")
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {camera_index}")
        return False
    
    print("Camera opened successfully!")
    print(f"Resolution: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
    print(f"FPS: {cap.get(cv2.CAP_PROP_FPS)}")
    
    print("\nPress 'q' to quit camera test")
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("ERROR: Failed to capture frame")
            break
        
        cv2.putText(
            frame, "Camera Test - Press 'q' to quit", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        
        cv2.imshow('Camera Test', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    print("Camera test completed successfully!")
    return True


if __name__ == '__main__':
    camera_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    test_camera(camera_idx)

