"""
Gesture Mapper - Maps detected gestures to robot commands
Handles different control modes and gesture priorities
"""

from typing import Dict, Optional, Tuple
from enum import Enum


class ControlMode(Enum):
    """Available control modes"""
    MOVEMENT = "movement"
    ADVANCED_MOVEMENT = "advanced_movement"
    GIMBAL = "gimbal"
    PRECISION = "precision"


class GestureMapper:
    """Maps gestures to robot commands based on current mode"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.current_mode = ControlMode.MOVEMENT
        self.speed_level = 3  # Default speed (1-5)
        
        # Define gesture mappings for each mode
        self._init_gesture_mappings()
        
        # Safety gestures (always active regardless of mode)
        self.safety_gestures = {
            "open_palm": "emergency_stop",
            "fist": "hold_position"
        }
    
    def _init_gesture_mappings(self):
        """Initialize gesture-to-command mappings for all modes"""
        
        # Movement Mode - Basic navigation
        self.movement_mappings = {
            "thumbs_up": "move_forward",
            "thumbs_down": "move_backward",
            "swipe_left": "turn_left",
            "swipe_right": "turn_right",
            "circle_clockwise": "rotate_clockwise",
            "circle_counter_clockwise": "rotate_counter_clockwise",
            "peace_sign": "switch_mode_gimbal",
            "pointing": "switch_mode_precision",
        }
        
        # Advanced Movement Mode - Speed and rotation control
        self.advanced_movement_mappings = {
            **self.movement_mappings,
            "swipe_up": "increase_speed",
            "swipe_down": "decrease_speed",
            "count_1": "set_speed_1",
            "count_2": "set_speed_2",
            "count_3": "set_speed_3",
            "count_4": "set_speed_4",
            "count_5": "set_speed_5",
        }
        
        # Gimbal Control Mode - Camera positioning
        self.gimbal_mappings = {
            "swipe_left": "gimbal_pan_left",
            "swipe_right": "gimbal_pan_right",
            "swipe_up": "gimbal_tilt_up",
            "swipe_down": "gimbal_tilt_down",
            "ok_sign": "gimbal_center",
            "wave": "switch_mode_movement",
        }
        
        # Precision Mode - Slow, precise movements
        self.precision_mappings = {
            "thumbs_up": "move_forward_slow",
            "thumbs_down": "move_backward_slow",
            "swipe_left": "turn_left_slow",
            "swipe_right": "turn_right_slow",
            "wave": "switch_mode_movement",
        }
    
    def map_gesture(self, static_gesture: Optional[str], 
                    dynamic_gesture: Optional[str]) -> Tuple[Optional[str], Dict]:
        """
        Map detected gestures to robot commands
        
        Args:
            static_gesture: Detected static gesture
            dynamic_gesture: Detected dynamic gesture
        
        Returns:
            command: Command string or None
            params: Dictionary of command parameters
        """
        
        # Priority 1: Safety gestures (always checked first)
        if static_gesture in self.safety_gestures:
            command = self.safety_gestures[static_gesture]
            return command, {"priority": "high", "mode": "safety"}
        
        # Priority 2: Check for mode switching
        mode_switch = self._check_mode_switch(static_gesture, dynamic_gesture)
        if mode_switch:
            return mode_switch, {"mode_change": True}
        
        # Priority 3: Mode-specific gestures
        gesture = dynamic_gesture if dynamic_gesture else static_gesture
        
        if gesture is None:
            return None, {}
        
        # Get appropriate mapping based on current mode
        if self.current_mode == ControlMode.MOVEMENT:
            mappings = self.movement_mappings
        elif self.current_mode == ControlMode.ADVANCED_MOVEMENT:
            mappings = self.advanced_movement_mappings
        elif self.current_mode == ControlMode.GIMBAL:
            mappings = self.gimbal_mappings
        elif self.current_mode == ControlMode.PRECISION:
            mappings = self.precision_mappings
        else:
            mappings = self.movement_mappings
        
        command = mappings.get(gesture)
        
        if command:
            params = self._get_command_params(command)
            return command, params
        
        return None, {}
    
    def _check_mode_switch(self, static_gesture: Optional[str], 
                          dynamic_gesture: Optional[str]) -> Optional[str]:
        """Check if gesture triggers mode switch"""
        
        if static_gesture == "peace_sign":
            self.current_mode = ControlMode.GIMBAL
            return "mode_switch_gimbal"
        
        elif static_gesture == "pointing":
            self.current_mode = ControlMode.PRECISION
            return "mode_switch_precision"
        
        elif dynamic_gesture == "wave":
            self.current_mode = ControlMode.MOVEMENT
            return "mode_switch_movement"
        
        return None
    
    def _get_command_params(self, command: str) -> Dict:
        """Get parameters for a command based on current state"""
        
        params = {
            "mode": self.current_mode.value,
            "speed_level": self.speed_level,
            "timestamp": None  # Will be set by the control node
        }
        
        # Speed commands
        if command.startswith("set_speed_"):
            speed = int(command.split("_")[-1])
            self.speed_level = speed
            params["new_speed"] = speed
        
        elif command == "increase_speed":
            self.speed_level = min(5, self.speed_level + 1)
            params["new_speed"] = self.speed_level
        
        elif command == "decrease_speed":
            self.speed_level = max(1, self.speed_level - 1)
            params["new_speed"] = self.speed_level
        
        # Get speed multipliers from config
        speed_multipliers = self.config.get("speed_multipliers", {
            1: 0.2, 2: 0.4, 3: 0.6, 4: 0.8, 5: 1.0
        })
        params["speed_multiplier"] = speed_multipliers.get(self.speed_level, 0.6)
        
        # Precision mode uses slower speeds
        if self.current_mode == ControlMode.PRECISION:
            params["speed_multiplier"] *= 0.5
        
        return params
    
    def get_current_mode(self) -> ControlMode:
        """Get current control mode"""
        return self.current_mode
    
    def get_speed_level(self) -> int:
        """Get current speed level"""
        return self.speed_level
    
    def reset(self):
        """Reset to default mode and speed"""
        self.current_mode = ControlMode.MOVEMENT
        self.speed_level = 3
