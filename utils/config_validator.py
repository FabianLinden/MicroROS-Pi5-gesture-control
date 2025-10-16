"""
Configuration Validator for Gesture Control System
Validates YAML configuration against expected schema
"""

from typing import Dict, List, Tuple, Any
import logging

logger = logging.getLogger('config_validator')


class ConfigValidator:
    """Validates configuration files against expected schema"""
    
    # Define expected configuration schema
    SCHEMA = {
        # Camera settings
        'camera_width': {'type': int, 'min': 160, 'max': 1920, 'required': True},
        'camera_height': {'type': int, 'min': 120, 'max': 1080, 'required': True},
        'fps': {'type': int, 'min': 1, 'max': 60, 'required': True},
        'show_visualization': {'type': bool, 'required': True},
        
        # Gesture detection settings
        'min_detection_confidence': {'type': float, 'min': 0.0, 'max': 1.0, 'required': True},
        'min_tracking_confidence': {'type': float, 'min': 0.0, 'max': 1.0, 'required': True},
        'gesture_history_size': {'type': int, 'min': 10, 'max': 100, 'required': True},
        'min_movement_threshold': {'type': float, 'min': 0.001, 'max': 1.0, 'required': True},
        'gesture_cooldown': {'type': float, 'min': 0.0, 'max': 5.0, 'required': True},
        'gesture_smoothing_window': {'type': int, 'min': 1, 'max': 10, 'required': False},
        'min_confidence_threshold': {'type': float, 'min': 0.0, 'max': 1.0, 'required': False},
        
        # ROS2 topics
        'ros_topics': {
            'type': dict,
            'required': True,
            'schema': {
                'cmd_vel': {'type': str, 'required': True},
                'servo_control': {'type': str, 'required': True},
                'gesture_status': {'type': str, 'required': True}
            }
        },
        
        # Robot limits
        'robot_limits': {
            'type': dict,
            'required': True,
            'schema': {
                'max_linear_speed': {'type': float, 'min': 0.0, 'max': 10.0, 'required': True},
                'max_angular_speed': {'type': float, 'min': 0.0, 'max': 10.0, 'required': True},
                'gimbal_step': {'type': float, 'min': 1.0, 'max': 90.0, 'required': True}
            }
        },
        
        # Speed multipliers
        'speed_multipliers': {
            'type': dict,
            'required': True,
            'schema': {
                1: {'type': float, 'min': 0.0, 'max': 1.0, 'required': True},
                2: {'type': float, 'min': 0.0, 'max': 1.0, 'required': True},
                3: {'type': float, 'min': 0.0, 'max': 1.0, 'required': True},
                4: {'type': float, 'min': 0.0, 'max': 1.0, 'required': True},
                5: {'type': float, 'min': 0.0, 'max': 1.0, 'required': True}
            }
        },
        
        # Safety settings
        'safety_timeout': {'type': float, 'min': 0.5, 'max': 10.0, 'required': True},
        
        # Logging settings
        'log_level': {
            'type': str,
            'required': False,
            'allowed_values': ['DEBUG', 'INFO', 'WARN', 'WARNING', 'ERROR', 'CRITICAL']
        }
    }
    
    @classmethod
    def validate(cls, config: Dict) -> Tuple[bool, List[str]]:
        """
        Validate configuration against schema
        
        Args:
            config: Configuration dictionary to validate
        
        Returns:
            is_valid: True if configuration is valid
            errors: List of error messages (empty if valid)
        """
        errors = []
        
        # Validate top-level configuration
        cls._validate_dict(config, cls.SCHEMA, '', errors)
        
        # Additional validation rules
        cls._validate_speed_multipliers(config, errors)
        
        is_valid = len(errors) == 0
        
        if is_valid:
            logger.info("Configuration validation passed")
        else:
            logger.error(f"Configuration validation failed with {len(errors)} errors")
            for error in errors:
                logger.error(f"  - {error}")
        
        return is_valid, errors
    
    @classmethod
    def _validate_dict(cls, config: Dict, schema: Dict, path: str, errors: List[str]):
        """Recursively validate dictionary against schema"""
        
        for key, rules in schema.items():
            current_path = f"{path}.{key}" if path else key
            
            # Check if required key is present
            if rules.get('required', False) and key not in config:
                errors.append(f"Missing required field: {current_path}")
                continue
            
            # Skip if optional and not present
            if key not in config:
                continue
            
            value = config[key]
            expected_type = rules.get('type')
            
            # Type validation
            if expected_type and not isinstance(value, expected_type):
                errors.append(
                    f"Invalid type for {current_path}: expected {expected_type.__name__}, "
                    f"got {type(value).__name__}"
                )
                continue
            
            # Nested dictionary validation
            if expected_type == dict and 'schema' in rules:
                cls._validate_dict(value, rules['schema'], current_path, errors)
                continue
            
            # Range validation for numbers
            if isinstance(value, (int, float)):
                if 'min' in rules and value < rules['min']:
                    errors.append(
                        f"Value for {current_path} ({value}) is below minimum ({rules['min']})"
                    )
                
                if 'max' in rules and value > rules['max']:
                    errors.append(
                        f"Value for {current_path} ({value}) is above maximum ({rules['max']})"
                    )
            
            # Allowed values validation
            if 'allowed_values' in rules and value not in rules['allowed_values']:
                errors.append(
                    f"Invalid value for {current_path}: {value}. "
                    f"Allowed values: {rules['allowed_values']}"
                )
    
    @classmethod
    def _validate_speed_multipliers(cls, config: Dict, errors: List[str]):
        """Validate speed multipliers are in ascending order"""
        
        if 'speed_multipliers' not in config:
            return
        
        multipliers = config['speed_multipliers']
        
        # Convert keys to integers and sort
        try:
            sorted_levels = sorted([int(k) for k in multipliers.keys()])
            
            # Check if multipliers are in ascending order
            prev_mult = 0.0
            for level in sorted_levels:
                mult = multipliers[level]
                if mult < prev_mult:
                    errors.append(
                        f"Speed multipliers should be in ascending order. "
                        f"Level {level} ({mult}) is less than previous level ({prev_mult})"
                    )
                prev_mult = mult
        
        except (ValueError, KeyError) as e:
            errors.append(f"Invalid speed_multipliers format: {e}")
    
    @classmethod
    def get_default_config(cls) -> Dict:
        """
        Generate a default configuration based on schema
        
        Returns:
            Default configuration dictionary
        """
        return {
            'camera_width': 640,
            'camera_height': 480,
            'fps': 30,
            'show_visualization': True,
            'min_detection_confidence': 0.7,
            'min_tracking_confidence': 0.5,
            'gesture_history_size': 30,
            'min_movement_threshold': 0.03,
            'gesture_cooldown': 0.5,
            'gesture_smoothing_window': 3,
            'min_confidence_threshold': 0.7,
            'ros_topics': {
                'cmd_vel': '/cmd_vel',
                'servo_control': '/servo_control',
                'gesture_status': '/gesture_status'
            },
            'robot_limits': {
                'max_linear_speed': 0.5,
                'max_angular_speed': 1.0,
                'gimbal_step': 10.0
            },
            'speed_multipliers': {
                1: 0.2,
                2: 0.4,
                3: 0.6,
                4: 0.8,
                5: 1.0
            },
            'safety_timeout': 2.0,
            'log_level': 'INFO'
        }


def validate_config_file(config_path: str) -> Tuple[bool, List[str], Dict]:
    """
    Load and validate a configuration file
    
    Args:
        config_path: Path to YAML configuration file
    
    Returns:
        is_valid: True if configuration is valid
        errors: List of error messages
        config: Loaded configuration dictionary
    """
    import yaml
    
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        is_valid, errors = ConfigValidator.validate(config)
        return is_valid, errors, config
    
    except FileNotFoundError:
        return False, [f"Configuration file not found: {config_path}"], {}
    
    except yaml.YAMLError as e:
        return False, [f"YAML parsing error: {e}"], {}
    
    except Exception as e:
        return False, [f"Unexpected error loading config: {e}"], {}


if __name__ == '__main__':
    # Test validation with default config
    import sys
    
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    else:
        config_path = '../config/gesture_config.yaml'
    
    print(f"Validating configuration: {config_path}")
    print("=" * 60)
    
    is_valid, errors, config = validate_config_file(config_path)
    
    if is_valid:
        print("✓ Configuration is valid!")
    else:
        print("✗ Configuration validation failed:")
        for error in errors:
            print(f"  - {error}")
        sys.exit(1)
