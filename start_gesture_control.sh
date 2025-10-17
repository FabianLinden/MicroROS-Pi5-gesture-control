#!/bin/bash

echo "🤖 MicroROS-Pi5 Gesture Control Launcher"
echo "========================================"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
echo "✓ ROS2 Humble sourced"

# Navigate to project
cd ~/repos/MicroROS-Pi5
echo "✓ Changed to project directory"

# Check if robot nodes are running
if ! ros2 node list &>/dev/null; then
    echo "⚠️  Warning: No ROS2 nodes detected!"
    echo "   Is your robot running?"
    read -p "   Continue anyway? (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "✓ ROS2 nodes detected"
echo ""
echo "🚀 Starting gesture control..."
echo "   Press Ctrl+C to stop"
echo "   Press 'q' in window to quit"
echo ""

# Run gesture control
uv run python main.py

# Cleanup message
echo ""
echo "👋 Gesture control stopped."
echo "   Robot has been stopped safely."


exit 0

