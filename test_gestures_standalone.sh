#!/bin/bash
# Test gesture detection without ROS2/Docker

echo "🖐️  Gesture Detection Test (No Robot Control)"
echo "=============================================="
echo ""
echo "This will test:"
echo "  ✓ Camera access"
echo "  ✓ Hand tracking"
echo "  ✓ Gesture recognition"
echo "  ✗ Robot control (disabled)"
echo ""
echo "Press 'q' in the window to quit"
echo ""

cd ~/MicroROS-Pi5-gesture-control

# Run test mode
uv run python main.py --test-gestures

echo ""
echo "Test complete!"
