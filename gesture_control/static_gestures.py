"""
Static Gesture Detection using MediaPipe
Recognizes hand shapes and poses
"""

import mediapipe as mp
import numpy as np
from typing import Optional, Dict, Tuple


class StaticGestureDetector:
    """Detects static hand gestures using MediaPipe hand landmarks"""
    
    def __init__(self, min_detection_confidence=0.7, min_tracking_confidence=0.5):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        self.mp_draw = mp.solutions.drawing_utils
        
    def detect(self, frame) -> Tuple[Optional[str], Optional[Dict], any]:
        """
        Detect static gesture in frame
        
        Returns:
            gesture_name: Name of detected gesture or None
            landmarks: Hand landmarks data
            results: MediaPipe results object
        """
        frame_rgb = frame.copy()
        results = self.hands.process(frame_rgb)
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            gesture, confidence = self._classify_gesture(hand_landmarks)
            
            # Add confidence to landmarks data
            landmarks_data = {
                'landmarks': hand_landmarks,
                'confidence': confidence
            }
            
            return gesture, landmarks_data, results
        
        return None, None, results
    
    def _classify_gesture(self, hand_landmarks) -> Tuple[Optional[str], float]:
        """
        Classify the hand gesture based on landmarks
        
        Returns:
            gesture_name: Name of detected gesture or None
            confidence: Confidence score (0.0 to 1.0)
        """
        
        # Extract landmark positions
        landmarks = hand_landmarks.landmark
        
        # Get finger states (extended or folded)
        fingers_up = self._count_fingers(landmarks)
        
        # Classify based on finger states and positions
        # Return gesture name and confidence score
        
        if fingers_up == [0, 0, 0, 0, 0]:
            return "fist", 0.95
        
        elif fingers_up == [1, 1, 1, 1, 1]:
            return "open_palm", 0.95
        
        elif fingers_up == [1, 0, 0, 0, 0]:
            # Check if thumb is up
            if self._is_thumb_up(landmarks):
                return "thumbs_up", 0.90
            elif self._is_thumb_down(landmarks):
                return "thumbs_down", 0.90
        
        elif fingers_up == [0, 1, 1, 0, 0]:
            return "peace_sign", 0.90
        
        elif fingers_up == [0, 1, 0, 0, 0]:
            return "pointing", 0.90
        
        elif self._is_ok_sign(landmarks, fingers_up):
            return "ok_sign", 0.85
        
        # Finger counting for speed levels
        elif sum(fingers_up) in [1, 2, 3, 4, 5]:
            count = sum(fingers_up)
            # Lower confidence for counting gestures as they can be ambiguous
            confidence = 0.80 if count in [1, 5] else 0.85
            return f"count_{count}", confidence
        
        return "unknown", 0.0
    
    def _count_fingers(self, landmarks) -> list:
        """
        Count which fingers are extended
        Returns: [thumb, index, middle, ring, pinky] - 1 if extended, 0 if folded
        """
        fingers = []
        
        # Thumb - check if tip is farther from wrist than IP joint
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_mcp = landmarks[self.mp_hands.HandLandmark.THUMB_MCP]
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        
        # Thumb extended if tip is farther from wrist than IP
        thumb_dist_tip = self._distance(thumb_tip, wrist)
        thumb_dist_ip = self._distance(thumb_ip, wrist)
        fingers.append(1 if thumb_dist_tip > thumb_dist_ip else 0)
        
        # Other fingers - check if tip is above PIP joint
        finger_tips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]
        
        finger_pips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP
        ]
        
        for tip, pip in zip(finger_tips, finger_pips):
            # Finger extended if tip y-coordinate is less than PIP (higher in image)
            fingers.append(1 if landmarks[tip].y < landmarks[pip].y else 0)
        
        return fingers
    
    def _is_thumb_up(self, landmarks) -> bool:
        """Check if thumb is pointing up"""
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        
        # Thumb up if tip is above IP and above index MCP
        return thumb_tip.y < thumb_ip.y and thumb_tip.y < index_mcp.y
    
    def _is_thumb_down(self, landmarks) -> bool:
        """Check if thumb is pointing down"""
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        
        # Thumb down if tip is below IP and below index MCP
        return thumb_tip.y > thumb_ip.y and thumb_tip.y > index_mcp.y
    
    def _is_ok_sign(self, landmarks, fingers_up) -> bool:
        """Check if making OK sign (thumb and index finger touching)"""
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        
        # Distance between thumb and index finger tips
        distance = self._distance(thumb_tip, index_tip)
        
        # OK sign if distance is small and other fingers are up
        return distance < 0.05 and fingers_up[2:] == [1, 1, 1]
    
    def _distance(self, point1, point2) -> float:
        """Calculate Euclidean distance between two landmarks"""
        return np.sqrt(
            (point1.x - point2.x) ** 2 +
            (point1.y - point2.y) ** 2 +
            (point1.z - point2.z) ** 2
        )
    
    def close(self):
        """Release resources"""
        self.hands.close()
