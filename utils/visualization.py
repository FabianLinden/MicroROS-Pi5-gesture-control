"""
Visualization utilities for gesture recognition debugging
"""

import cv2
import numpy as np
import mediapipe as mp
from typing import Dict, Optional


class GestureVisualizer:
    """Visualizes gesture detection results on video frames"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.vis_config = config.get('visualization', {})
        
        # MediaPipe drawing utilities
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Colors
        self.text_color = tuple(self.vis_config.get('text_color', [0, 255, 0]))
        self.landmark_color = tuple(self.vis_config.get('landmark_color', [0, 0, 255]))
        self.connection_color = tuple(self.vis_config.get('connection_color', [0, 255, 0]))
        
        # Font settings
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = self.vis_config.get('font_scale', 0.7)
        self.font_thickness = self.vis_config.get('font_thickness', 2)
    
    def draw(self, frame: np.ndarray, debug_info: Dict) -> np.ndarray:
        """
        Draw visualization overlays on frame
        
        Args:
            frame: Video frame (BGR)
            debug_info: Debug information from gesture recognizer
        
        Returns:
            Annotated frame
        """
        
        vis_frame = frame.copy()
        
        # Draw hand landmarks
        if self.vis_config.get('show_landmarks', True):
            self._draw_landmarks(vis_frame, debug_info)
        
        # Draw gesture information
        if self.vis_config.get('show_gesture_name', True):
            self._draw_gestures(vis_frame, debug_info)
        
        # Draw command information
        if self.vis_config.get('show_command', True):
            self._draw_command(vis_frame, debug_info)
        
        # Draw mode and speed level
        if self.vis_config.get('show_mode', True):
            self._draw_mode_info(vis_frame, debug_info)
        
        # Draw help text
        self._draw_help(vis_frame)
        
        return vis_frame
    
    def _draw_landmarks(self, frame: np.ndarray, debug_info: Dict):
        """Draw hand landmarks on frame"""
        
        mp_results = debug_info.get('mp_results')
        
        if mp_results and mp_results.multi_hand_landmarks:
            for hand_landmarks in mp_results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
    
    def _draw_gestures(self, frame: np.ndarray, debug_info: Dict):
        """Draw detected gesture names with confidence scores"""
        
        y_offset = 30
        
        static_gesture = debug_info.get('static_gesture')
        static_conf = debug_info.get('static_confidence', 0.0)
        
        if static_gesture:
            text = f"Static: {static_gesture} ({static_conf:.2f})"
            cv2.putText(
                frame, text, (10, y_offset),
                self.font, self.font_scale, self.text_color, self.font_thickness
            )
            y_offset += 30
        
        dynamic_gesture = debug_info.get('dynamic_gesture')
        dynamic_conf = debug_info.get('dynamic_confidence', 0.0)
        
        if dynamic_gesture:
            text = f"Dynamic: {dynamic_gesture} ({dynamic_conf:.2f})"
            cv2.putText(
                frame, text, (10, y_offset),
                self.font, self.font_scale, (255, 255, 0), self.font_thickness
            )
            y_offset += 30
    
    def _draw_command(self, frame: np.ndarray, debug_info: Dict):
        """Draw current command"""
        
        command = debug_info.get('command')
        
        if command:
            # Draw command in a box at the top center
            text = f"Command: {command}"
            text_size = cv2.getTextSize(text, self.font, self.font_scale, self.font_thickness)[0]
            
            # Calculate position for centered text
            text_x = (frame.shape[1] - text_size[0]) // 2
            text_y = 40
            
            # Draw background rectangle
            padding = 10
            cv2.rectangle(
                frame,
                (text_x - padding, text_y - text_size[1] - padding),
                (text_x + text_size[0] + padding, text_y + padding),
                (0, 0, 0),
                -1
            )
            
            # Draw text
            cv2.putText(
                frame, text, (text_x, text_y),
                self.font, self.font_scale, (0, 255, 255), self.font_thickness
            )
    
    def _draw_mode_info(self, frame: np.ndarray, debug_info: Dict):
        """Draw current mode and speed level"""
        
        mode = debug_info.get('mode')
        speed_level = debug_info.get('speed_level')
        
        # Draw in bottom left corner
        y_offset = frame.shape[0] - 60
        
        if mode:
            mode_text = f"Mode: {mode.value if hasattr(mode, 'value') else mode}"
            cv2.putText(
                frame, mode_text, (10, y_offset),
                self.font, self.font_scale, (255, 200, 0), self.font_thickness
            )
            y_offset += 30
        
        if speed_level:
            speed_text = f"Speed: {speed_level}/5"
            # Draw speed bar
            bar_width = 150
            bar_height = 20
            bar_x = 10
            bar_y = y_offset - 15
            
            # Background
            cv2.rectangle(
                frame,
                (bar_x, bar_y),
                (bar_x + bar_width, bar_y + bar_height),
                (50, 50, 50),
                -1
            )
            
            # Speed level fill
            fill_width = int(bar_width * (speed_level / 5))
            cv2.rectangle(
                frame,
                (bar_x, bar_y),
                (bar_x + fill_width, bar_y + bar_height),
                (0, 255, 0),
                -1
            )
            
            # Text
            cv2.putText(
                frame, speed_text, (bar_x + bar_width + 10, bar_y + 15),
                self.font, 0.5, (255, 200, 0), 1
            )
    
    def _draw_help(self, frame: np.ndarray):
        """Draw help text"""
        
        help_text = "Press 'q' to quit"
        text_size = cv2.getTextSize(help_text, self.font, 0.5, 1)[0]
        
        x = frame.shape[1] - text_size[0] - 10
        y = frame.shape[0] - 10
        
        cv2.putText(
            frame, help_text, (x, y),
            self.font, 0.5, (200, 200, 200), 1
        )

