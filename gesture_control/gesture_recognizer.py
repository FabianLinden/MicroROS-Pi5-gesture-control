"""
Main Gesture Recognizer - Coordinates static and dynamic gesture detection
"""

import cv2
import time
from typing import Optional, Dict, Tuple, List
from collections import deque
from .static_gestures import StaticGestureDetector
from .dynamic_gestures import DynamicGestureDetector
from .gesture_mapper import GestureMapper


class GestureRecognizer:
    """Main gesture recognition coordinator"""
    
    def __init__(self, config: Dict):
        self.config = config
        
        # Initialize detectors
        self.static_detector = StaticGestureDetector(
            min_detection_confidence=config.get("min_detection_confidence", 0.7),
            min_tracking_confidence=config.get("min_tracking_confidence", 0.5)
        )
        
        self.dynamic_detector = DynamicGestureDetector(
            history_size=config.get("gesture_history_size", 30),
            min_movement_threshold=config.get("min_movement_threshold", 0.03)
        )
        
        self.gesture_mapper = GestureMapper(config)
        
        # Gesture cooldown to prevent rapid triggering
        self.last_gesture_time = {}
        self.cooldown_period = config.get("gesture_cooldown", 0.5)  # seconds
        
        # Gesture smoothing - voting mechanism over multiple frames
        self.smoothing_window = config.get("gesture_smoothing_window", 3)
        self.static_gesture_history = deque(maxlen=self.smoothing_window)
        self.min_confidence_threshold = config.get("min_confidence_threshold", 0.7)
        
        # Current gesture state
        self.current_static_gesture = None
        self.current_dynamic_gesture = None
        self.last_command = None
        self.gesture_confidence = 0.0
        self.static_confidence = 0.0
        self.dynamic_confidence = 0.0
    
    def process_frame(self, frame) -> Tuple[Optional[str], Dict, any]:
        """
        Process a video frame and detect gestures
        
        Args:
            frame: Video frame (BGR format from OpenCV)
        
        Returns:
            command: Robot command string or None
            params: Command parameters dictionary
            debug_info: Information for visualization
        """
        
        # Convert to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Detect static gesture with confidence
        static_gesture, hand_landmarks, mp_results = self.static_detector.detect(frame_rgb)
        
        # Extract confidence from landmarks data
        if hand_landmarks and isinstance(hand_landmarks, dict):
            self.static_confidence = hand_landmarks.get('confidence', 0.0)
        else:
            self.static_confidence = 0.0
        
        # Update dynamic gesture detector with confidence
        dynamic_gesture = None
        self.dynamic_confidence = 0.0
        
        if hand_landmarks:
            dynamic_gesture, self.dynamic_confidence = self.dynamic_detector.update(
                hand_landmarks, 
                frame.shape
            )
        else:
            self.dynamic_detector.reset()
            self.static_gesture_history.clear()
        
        # Apply gesture smoothing for static gestures
        static_gesture = self._apply_smoothing(static_gesture, self.static_confidence)
        
        # Filter by confidence threshold
        if static_gesture and self.static_confidence < self.min_confidence_threshold:
            static_gesture = None
        
        if dynamic_gesture and self.dynamic_confidence < self.min_confidence_threshold:
            dynamic_gesture = None
        
        # Apply cooldown filter
        static_gesture = self._apply_cooldown(static_gesture, "static")
        dynamic_gesture = self._apply_cooldown(dynamic_gesture, "dynamic")
        
        # Store current gestures
        self.current_static_gesture = static_gesture
        self.current_dynamic_gesture = dynamic_gesture
        
        # Map gestures to commands
        command, params = self.gesture_mapper.map_gesture(
            static_gesture,
            dynamic_gesture
        )
        
        if command:
            self.last_command = command
            self.last_gesture_time[command] = time.time()
        
        # Prepare debug info for visualization
        debug_info = {
            "static_gesture": static_gesture,
            "dynamic_gesture": dynamic_gesture,
            "command": command,
            "params": params,
            "hand_landmarks": hand_landmarks,
            "mp_results": mp_results,
            "mode": self.gesture_mapper.get_current_mode(),
            "speed_level": self.gesture_mapper.get_speed_level(),
            "static_confidence": self.static_confidence,
            "dynamic_confidence": self.dynamic_confidence
        }
        
        return command, params, debug_info
    
    def _apply_smoothing(self, gesture: Optional[str], confidence: float) -> Optional[str]:
        """
        Apply voting mechanism over multiple frames to reduce false positives
        
        Args:
            gesture: Detected gesture name
            confidence: Confidence score for the gesture
        
        Returns:
            Smoothed gesture name or None
        """
        
        # Add current gesture to history
        self.static_gesture_history.append((gesture, confidence))
        
        # Need enough history for voting
        if len(self.static_gesture_history) < self.smoothing_window:
            return None
        
        # Count votes for each gesture
        gesture_votes = {}
        total_confidence = {}
        
        for gest, conf in self.static_gesture_history:
            if gest is not None:
                gesture_votes[gest] = gesture_votes.get(gest, 0) + 1
                total_confidence[gest] = total_confidence.get(gest, 0.0) + conf
        
        # No gestures detected
        if not gesture_votes:
            return None
        
        # Find gesture with most votes
        max_votes = max(gesture_votes.values())
        
        # Require majority (more than half of window)
        if max_votes > self.smoothing_window / 2:
            # Get gesture with most votes
            winning_gesture = max(gesture_votes, key=gesture_votes.get)
            
            # Calculate average confidence
            avg_confidence = total_confidence[winning_gesture] / gesture_votes[winning_gesture]
            
            # Only return if confidence is high enough
            if avg_confidence >= self.min_confidence_threshold:
                return winning_gesture
        
        return None
    
    def _apply_cooldown(self, gesture: Optional[str], gesture_type: str) -> Optional[str]:
        """Apply cooldown period to prevent gesture spam"""
        
        if gesture is None:
            return None
        
        gesture_key = f"{gesture_type}_{gesture}"
        current_time = time.time()
        
        # Check if gesture is in cooldown
        if gesture_key in self.last_gesture_time:
            time_since_last = current_time - self.last_gesture_time[gesture_key]
            if time_since_last < self.cooldown_period:
                return None
        
        return gesture
    
    def reset(self):
        """Reset all gesture detection state"""
        self.dynamic_detector.reset()
        self.gesture_mapper.reset()
        self.last_gesture_time.clear()
        self.static_gesture_history.clear()
        self.current_static_gesture = None
        self.current_dynamic_gesture = None
        self.last_command = None
        self.static_confidence = 0.0
        self.dynamic_confidence = 0.0
    
    def close(self):
        """Release resources"""
        self.static_detector.close()

