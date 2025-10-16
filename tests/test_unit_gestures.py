"""
Unit tests for gesture detection functions
"""

import pytest
import numpy as np
from unittest.mock import Mock, MagicMock
import sys
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from gesture_control.static_gestures import StaticGestureDetector
from gesture_control.dynamic_gestures import DynamicGestureDetector
from gesture_control.gesture_mapper import GestureMapper, ControlMode


class TestStaticGestureDetector:
    """Test static gesture detection"""
    
    def setup_method(self):
        """Setup test fixtures"""
        self.detector = StaticGestureDetector(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
    
    def teardown_method(self):
        """Cleanup after tests"""
        self.detector.close()
    
    def test_initialization(self):
        """Test detector initializes correctly"""
        assert self.detector is not None
        assert self.detector.mp_hands is not None
        assert self.detector.hands is not None
    
    def test_finger_counting_all_down(self):
        """Test finger counting with all fingers down (fist)"""
        # Create mock landmarks for fist
        mock_landmarks = self._create_mock_landmarks_fist()
        fingers = self.detector._count_fingers(mock_landmarks)
        
        # All fingers should be down
        assert fingers == [0, 0, 0, 0, 0]
    
    def test_finger_counting_all_up(self):
        """Test finger counting with all fingers up (open palm)"""
        # Create mock landmarks for open palm
        mock_landmarks = self._create_mock_landmarks_open_palm()
        fingers = self.detector._count_fingers(mock_landmarks)
        
        # All fingers should be up
        assert fingers == [1, 1, 1, 1, 1]
    
    def test_distance_calculation(self):
        """Test Euclidean distance calculation"""
        point1 = Mock()
        point1.x, point1.y, point1.z = 0.0, 0.0, 0.0
        
        point2 = Mock()
        point2.x, point2.y, point2.z = 3.0, 4.0, 0.0
        
        distance = self.detector._distance(point1, point2)
        
        # Should be 5.0 (3-4-5 triangle)
        assert abs(distance - 5.0) < 0.001
    
    def _create_mock_landmarks_fist(self):
        """Create mock landmarks for fist gesture"""
        landmarks = []
        
        # Create 21 landmarks (MediaPipe hand has 21 landmarks)
        for i in range(21):
            landmark = Mock()
            # All fingertips close to wrist (fist position)
            landmark.x = 0.5
            landmark.y = 0.5
            landmark.z = 0.0
            landmarks.append(landmark)
        
        return landmarks
    
    def _create_mock_landmarks_open_palm(self):
        """Create mock landmarks for open palm gesture"""
        landmarks = []
        
        # Create 21 landmarks with fingers extended
        for i in range(21):
            landmark = Mock()
            landmark.x = 0.5
            # Fingertips higher (lower y value) than base
            landmark.y = 0.3 if i in [4, 8, 12, 16, 20] else 0.5
            landmark.z = 0.0
            landmarks.append(landmark)
        
        return landmarks


class TestDynamicGestureDetector:
    """Test dynamic gesture detection"""
    
    def setup_method(self):
        """Setup test fixtures"""
        self.detector = DynamicGestureDetector(
            history_size=30,
            min_movement_threshold=0.03
        )
    
    def test_initialization(self):
        """Test detector initializes correctly"""
        assert self.detector is not None
        assert len(self.detector.position_history) == 0
        assert len(self.detector.centroid_history) == 0
    
    def test_swipe_right_detection(self):
        """Test swipe right gesture detection"""
        # Simulate hand moving right
        mock_landmarks = self._create_mock_hand_landmarks()
        
        # Add positions moving right
        for i in range(20):
            # Update x position to simulate rightward movement
            mock_landmarks.landmark[8].x = 0.3 + (i * 0.02)
            mock_landmarks.landmark[8].y = 0.5
            
            gesture, confidence = self.detector.update(
                {'landmarks': mock_landmarks},
                (480, 640)
            )
        
        # Should detect swipe_right
        assert gesture == "swipe_right"
        assert confidence > 0.7
    
    def test_swipe_left_detection(self):
        """Test swipe left gesture detection"""
        # Simulate hand moving left
        mock_landmarks = self._create_mock_hand_landmarks()
        
        # Add positions moving left
        for i in range(20):
            mock_landmarks.landmark[8].x = 0.7 - (i * 0.02)
            mock_landmarks.landmark[8].y = 0.5
            
            gesture, confidence = self.detector.update(
                {'landmarks': mock_landmarks},
                (480, 640)
            )
        
        # Should detect swipe_left
        assert gesture == "swipe_left"
        assert confidence > 0.7
    
    def test_insufficient_history(self):
        """Test that no gesture is detected with insufficient history"""
        mock_landmarks = self._create_mock_hand_landmarks()
        
        # Add only a few positions
        for i in range(5):
            gesture, confidence = self.detector.update(
                {'landmarks': mock_landmarks},
                (480, 640)
            )
        
        # Should not detect any gesture yet
        assert gesture is None
        assert confidence == 0.0
    
    def test_reset(self):
        """Test reset clears history"""
        mock_landmarks = self._create_mock_hand_landmarks()
        
        # Add some positions
        for i in range(10):
            self.detector.update(
                {'landmarks': mock_landmarks},
                (480, 640)
            )
        
        # Reset
        self.detector.reset()
        
        # History should be cleared
        assert len(self.detector.position_history) == 0
        assert len(self.detector.centroid_history) == 0
    
    def _create_mock_hand_landmarks(self):
        """Create mock hand landmarks"""
        mock_landmarks = Mock()
        mock_landmarks.landmark = []
        
        # Create 21 landmarks
        for i in range(21):
            landmark = Mock()
            landmark.x = 0.5
            landmark.y = 0.5
            landmark.z = 0.0
            mock_landmarks.landmark.append(landmark)
        
        return mock_landmarks


class TestGestureMapper:
    """Test gesture mapping to commands"""
    
    def setup_method(self):
        """Setup test fixtures"""
        self.config = {
            'speed_multipliers': {
                1: 0.2, 2: 0.4, 3: 0.6, 4: 0.8, 5: 1.0
            }
        }
        self.mapper = GestureMapper(self.config)
    
    def test_initialization(self):
        """Test mapper initializes correctly"""
        assert self.mapper is not None
        assert self.mapper.current_mode == ControlMode.MOVEMENT
        assert self.mapper.speed_level == 3
    
    def test_safety_gesture_priority(self):
        """Test safety gestures have highest priority"""
        # Emergency stop should override any other gesture
        command, params = self.mapper.map_gesture("open_palm", "swipe_left")
        
        assert command == "emergency_stop"
        assert params.get("priority") == "high"
        assert params.get("mode") == "safety"
    
    def test_movement_mode_mapping(self):
        """Test gesture mapping in movement mode"""
        self.mapper.current_mode = ControlMode.MOVEMENT
        
        # Test thumbs up -> move forward
        command, params = self.mapper.map_gesture("thumbs_up", None)
        assert command == "move_forward"
        
        # Test swipe left -> turn left
        command, params = self.mapper.map_gesture(None, "swipe_left")
        assert command == "turn_left"
    
    def test_gimbal_mode_mapping(self):
        """Test gesture mapping in gimbal mode"""
        self.mapper.current_mode = ControlMode.GIMBAL
        
        # Test swipe left -> gimbal pan left
        command, params = self.mapper.map_gesture(None, "swipe_left")
        assert command == "gimbal_pan_left"
        
        # Test OK sign -> center gimbal
        command, params = self.mapper.map_gesture("ok_sign", None)
        assert command == "gimbal_center"
    
    def test_precision_mode_mapping(self):
        """Test gesture mapping in precision mode"""
        self.mapper.current_mode = ControlMode.PRECISION
        
        # Test thumbs up -> slow forward
        command, params = self.mapper.map_gesture("thumbs_up", None)
        assert command == "move_forward_slow"
        
        # Speed should be halved in precision mode
        assert params.get("speed_multiplier") == 0.3  # 0.6 * 0.5
    
    def test_mode_switching(self):
        """Test mode switching gestures"""
        # Start in movement mode
        assert self.mapper.current_mode == ControlMode.MOVEMENT
        
        # Switch to gimbal mode
        command, params = self.mapper.map_gesture("peace_sign", None)
        assert command == "mode_switch_gimbal"
        assert self.mapper.current_mode == ControlMode.GIMBAL
        
        # Switch to precision mode
        command, params = self.mapper.map_gesture("pointing", None)
        assert command == "mode_switch_precision"
        assert self.mapper.current_mode == ControlMode.PRECISION
        
        # Switch back to movement mode
        command, params = self.mapper.map_gesture(None, "wave")
        assert command == "mode_switch_movement"
        assert self.mapper.current_mode == ControlMode.MOVEMENT
    
    def test_speed_adjustment(self):
        """Test speed level adjustment"""
        # Set speed to level 1
        command, params = self.mapper.map_gesture("count_1", None)
        assert command == "set_speed_1"
        assert self.mapper.speed_level == 1
        assert params.get("speed_multiplier") == 0.2
        
        # Increase speed
        command, params = self.mapper.map_gesture(None, "swipe_up")
        assert command == "increase_speed"
        assert self.mapper.speed_level == 2
        
        # Decrease speed
        command, params = self.mapper.map_gesture(None, "swipe_down")
        assert command == "decrease_speed"
        assert self.mapper.speed_level == 1
    
    def test_speed_limits(self):
        """Test speed level stays within bounds"""
        # Set to max speed
        self.mapper.speed_level = 5
        
        # Try to increase beyond max
        command, params = self.mapper.map_gesture(None, "swipe_up")
        assert self.mapper.speed_level == 5  # Should stay at 5
        
        # Set to min speed
        self.mapper.speed_level = 1
        
        # Try to decrease below min
        command, params = self.mapper.map_gesture(None, "swipe_down")
        assert self.mapper.speed_level == 1  # Should stay at 1
    
    def test_reset(self):
        """Test reset returns to defaults"""
        # Change mode and speed
        self.mapper.current_mode = ControlMode.GIMBAL
        self.mapper.speed_level = 5
        
        # Reset
        self.mapper.reset()
        
        # Should be back to defaults
        assert self.mapper.current_mode == ControlMode.MOVEMENT
        assert self.mapper.speed_level == 3


class TestConfigValidator:
    """Test configuration validation"""
    
    def test_valid_config(self):
        """Test validation passes for valid config"""
        from utils.config_validator import ConfigValidator
        
        config = ConfigValidator.get_default_config()
        is_valid, errors = ConfigValidator.validate(config)
        
        assert is_valid
        assert len(errors) == 0
    
    def test_missing_required_field(self):
        """Test validation fails for missing required field"""
        from utils.config_validator import ConfigValidator
        
        config = ConfigValidator.get_default_config()
        del config['camera_width']
        
        is_valid, errors = ConfigValidator.validate(config)
        
        assert not is_valid
        assert any('camera_width' in error for error in errors)
    
    def test_invalid_type(self):
        """Test validation fails for invalid type"""
        from utils.config_validator import ConfigValidator
        
        config = ConfigValidator.get_default_config()
        config['camera_width'] = "640"  # Should be int, not string
        
        is_valid, errors = ConfigValidator.validate(config)
        
        assert not is_valid
        assert any('camera_width' in error and 'type' in error for error in errors)
    
    def test_value_out_of_range(self):
        """Test validation fails for out of range values"""
        from utils.config_validator import ConfigValidator
        
        config = ConfigValidator.get_default_config()
        config['min_detection_confidence'] = 1.5  # Should be 0.0-1.0
        
        is_valid, errors = ConfigValidator.validate(config)
        
        assert not is_valid
        assert any('min_detection_confidence' in error for error in errors)
    
    def test_invalid_log_level(self):
        """Test validation fails for invalid log level"""
        from utils.config_validator import ConfigValidator
        
        config = ConfigValidator.get_default_config()
        config['log_level'] = "INVALID"
        
        is_valid, errors = ConfigValidator.validate(config)
        
        assert not is_valid
        assert any('log_level' in error for error in errors)


if __name__ == '__main__':
    # Run tests with pytest
    pytest.main([__file__, '-v'])
