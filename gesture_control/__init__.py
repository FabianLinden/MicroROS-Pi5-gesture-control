"""
Gesture Control Package for MicroROS-Pi5
Provides static and dynamic hand gesture recognition using MediaPipe and OpenCV
"""

from .gesture_recognizer import GestureRecognizer
from .static_gestures import StaticGestureDetector
from .dynamic_gestures import DynamicGestureDetector
from .gesture_mapper import GestureMapper

__all__ = [
    'GestureRecognizer',
    'StaticGestureDetector',
    'DynamicGestureDetector',
    'GestureMapper',
]

__version__ = '1.0.0'
