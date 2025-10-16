"""
Dynamic Gesture Detection using OpenCV
Tracks hand movement over time to detect dynamic gestures
"""

import cv2
import numpy as np
from typing import Optional, List, Tuple
from collections import deque
import math


class DynamicGestureDetector:
    """Detects dynamic hand gestures by tracking movement over time"""
    
    def __init__(self, history_size=30, min_movement_threshold=0.03):
        self.history_size = history_size
        self.min_movement_threshold = min_movement_threshold
        
        # Movement history buffers
        self.position_history = deque(maxlen=history_size)
        self.centroid_history = deque(maxlen=history_size)
        
        # Gesture detection state
        self.current_gesture = None
        self.gesture_confidence = 0.0
        self.frame_count = 0
        
    def update(self, hand_landmarks, frame_shape) -> Tuple[Optional[str], float]:
        """
        Update with new hand position and detect dynamic gestures
        
        Args:
            hand_landmarks: MediaPipe hand landmarks (dict with 'landmarks' key)
            frame_shape: Shape of the video frame (height, width)
        
        Returns:
            Detected dynamic gesture name or None
            Confidence score (0.0 to 1.0)
        """
        if hand_landmarks is None:
            self.position_history.clear()
            self.centroid_history.clear()
            return None, 0.0
        
        # Extract actual landmarks from dict
        actual_landmarks = hand_landmarks.get('landmarks') if isinstance(hand_landmarks, dict) else hand_landmarks
        
        # Calculate hand centroid
        centroid = self._calculate_centroid(actual_landmarks, frame_shape)
        self.centroid_history.append(centroid)
        
        # Get index finger tip position (used for pointing gestures)
        index_tip = actual_landmarks.landmark[8]  # INDEX_FINGER_TIP
        position = np.array([index_tip.x, index_tip.y])
        self.position_history.append(position)
        
        # Need sufficient history to detect gestures
        if len(self.position_history) < 10:
            return None, 0.0
        
        # Detect various dynamic gestures
        gesture, confidence = self._detect_gesture()
        self.gesture_confidence = confidence
        return gesture, confidence
    
    def _calculate_centroid(self, hand_landmarks, frame_shape) -> np.ndarray:
        """Calculate the centroid of the hand"""
        x_coords = [lm.x for lm in hand_landmarks.landmark]
        y_coords = [lm.y for lm in hand_landmarks.landmark]
        
        centroid_x = np.mean(x_coords)
        centroid_y = np.mean(y_coords)
        
        return np.array([centroid_x, centroid_y])
    
    def _detect_gesture(self) -> Tuple[Optional[str], float]:
        """
        Detect dynamic gesture from movement history
        
        Returns:
            gesture_name: Name of detected gesture or None
            confidence: Confidence score (0.0 to 1.0)
        """
        
        # Try detecting different gesture types
        # Priority order: swipe > circle > wave > push/pull > pinch/spread
        
        # Swipe gestures (using index finger tracking)
        swipe, swipe_conf = self._detect_swipe()
        if swipe:
            return swipe, swipe_conf
        
        # Circular gestures (using centroid)
        circle, circle_conf = self._detect_circle()
        if circle:
            return circle, circle_conf
        
        # Wave gesture
        wave, wave_conf = self._detect_wave()
        if wave:
            return wave, wave_conf
        
        # Push/Pull gestures
        push_pull, pp_conf = self._detect_push_pull()
        if push_pull:
            return push_pull, pp_conf
        
        # Pinch/Spread gestures
        pinch_spread, ps_conf = self._detect_pinch_spread()
        if pinch_spread:
            return pinch_spread, ps_conf
        
        return None, 0.0
    
    def _detect_swipe(self) -> Tuple[Optional[str], float]:
        """
        Detect swipe left/right/up/down gestures
        
        Returns:
            gesture_name: Name of swipe gesture or None
            confidence: Confidence score based on movement clarity
        """
        if len(self.position_history) < 15:
            return None, 0.0
        
        # Get recent movement vector
        start_pos = self.position_history[-15]
        end_pos = self.position_history[-1]
        movement = end_pos - start_pos
        
        # Calculate movement magnitude and direction
        magnitude = np.linalg.norm(movement)
        
        if magnitude < self.min_movement_threshold:
            return None, 0.0
        
        # Determine primary direction
        dx, dy = movement
        
        # Calculate confidence based on directional clarity
        # Higher ratio = more confident in direction
        
        # Horizontal swipe
        if abs(dx) > abs(dy) * 1.5:
            ratio = abs(dx) / (abs(dy) + 0.001)  # Avoid division by zero
            confidence = min(0.95, 0.70 + (ratio - 1.5) * 0.1)
            
            if dx > 0:
                return "swipe_right", confidence
            else:
                return "swipe_left", confidence
        
        # Vertical swipe
        elif abs(dy) > abs(dx) * 1.5:
            ratio = abs(dy) / (abs(dx) + 0.001)
            confidence = min(0.95, 0.70 + (ratio - 1.5) * 0.1)
            
            if dy > 0:
                return "swipe_down", confidence
            else:
                return "swipe_up", confidence
        
        return None, 0.0
    
    def _detect_circle(self) -> Tuple[Optional[str], float]:
        """
        Detect circular motion (clockwise or counter-clockwise)
        
        Returns:
            gesture_name: Circle gesture or None
            confidence: Based on rotation amount and smoothness
        """
        if len(self.centroid_history) < 20:
            return None, 0.0
        
        # Get recent positions
        positions = list(self.centroid_history)[-20:]
        
        # Calculate center of the circular path
        center = np.mean(positions, axis=0)
        
        # Calculate angles from center
        angles = []
        for pos in positions:
            vec = pos - center
            angle = math.atan2(vec[1], vec[0])
            angles.append(angle)
        
        # Calculate total angle change
        total_rotation = 0
        for i in range(1, len(angles)):
            angle_diff = angles[i] - angles[i-1]
            
            # Normalize angle difference to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            total_rotation += angle_diff
        
        # Check if enough rotation occurred
        min_rotation = math.pi * 0.75  # At least 135 degrees
        
        if abs(total_rotation) > min_rotation:
            # Confidence increases with more rotation (up to 360 degrees)
            rotation_ratio = min(abs(total_rotation) / (2 * math.pi), 1.0)
            confidence = 0.75 + (rotation_ratio * 0.20)
            
            if total_rotation > 0:
                return "circle_clockwise", confidence
            else:
                return "circle_counter_clockwise", confidence
        
        return None, 0.0
    
    def _detect_wave(self) -> Tuple[Optional[str], float]:
        """
        Detect wave gesture (repeated left-right motion)
        
        Returns:
            gesture_name: "wave" or None
            confidence: Based on number of oscillations
        """
        if len(self.position_history) < 25:
            return None, 0.0
        
        # Check for oscillating motion
        positions = np.array(list(self.position_history)[-25:])
        x_positions = positions[:, 0]
        
        # Count direction changes
        direction_changes = 0
        for i in range(2, len(x_positions)):
            prev_dir = x_positions[i-1] - x_positions[i-2]
            curr_dir = x_positions[i] - x_positions[i-1]
            
            if prev_dir * curr_dir < 0:  # Sign change
                direction_changes += 1
        
        # Wave if multiple direction changes
        if direction_changes >= 3:
            # More oscillations = higher confidence
            confidence = min(0.95, 0.70 + (direction_changes - 3) * 0.05)
            return "wave", confidence
        
        return None, 0.0
    
    def _detect_push_pull(self) -> Tuple[Optional[str], float]:
        """
        Detect push (forward) or pull (backward) motion
        
        Returns:
            gesture_name: Push/pull gesture or None
            confidence: Based on movement magnitude
        """
        if len(self.centroid_history) < 15:
            return None, 0.0
        
        # Analyze z-depth change (simulated by hand size change)
        # In 2D, we approximate this by checking if hand moves toward/away from center
        
        start_pos = self.centroid_history[-15]
        end_pos = self.centroid_history[-1]
        
        # Movement toward/away from screen center
        center = np.array([0.5, 0.5])
        
        start_dist = np.linalg.norm(start_pos - center)
        end_dist = np.linalg.norm(end_pos - center)
        
        dist_change = end_dist - start_dist
        
        # Significant movement toward center = push, away = pull
        threshold = 0.08
        if abs(dist_change) > threshold:
            # Confidence based on how much beyond threshold
            confidence = min(0.90, 0.70 + (abs(dist_change) - threshold) * 2.0)
            
            if dist_change < 0:
                return "push_forward", confidence
            else:
                return "pull_backward", confidence
        
        return None, 0.0
    
    def _detect_pinch_spread(self) -> Tuple[Optional[str], float]:
        """
        Detect pinch (fingers coming together) or spread (fingers apart) gesture
        
        Returns:
            gesture_name: Pinch/spread gesture or None
            confidence: Based on variance change magnitude
        """
        # This requires tracking multiple fingertips
        # For simplicity, we'll track the variance in position history
        
        if len(self.position_history) < 15:
            return None, 0.0
        
        # Calculate variance at start and end of window
        recent_positions = list(self.position_history)
        
        start_window = recent_positions[:5]
        end_window = recent_positions[-5:]
        
        start_variance = np.var(start_window, axis=0).sum()
        end_variance = np.var(end_window, axis=0).sum()
        
        variance_change = end_variance - start_variance
        
        # Significant change in variance indicates pinch/spread
        threshold = 0.001
        if abs(variance_change) > threshold:
            # Confidence based on variance change magnitude
            confidence = min(0.85, 0.65 + (abs(variance_change) - threshold) * 100)
            
            if variance_change < 0:
                return "pinch", confidence
            else:
                return "spread", confidence
        
        return None, 0.0
    
    def reset(self):
        """Reset gesture detection state"""
        self.position_history.clear()
        self.centroid_history.clear()
        self.current_gesture = None
        self.gesture_confidence = 0.0

