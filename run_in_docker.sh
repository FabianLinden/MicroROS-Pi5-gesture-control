#!/bin/bash
# Helper script to run gesture control inside the ROS2 Docker container

echo "🤖 MicroROS-Pi5 Gesture Control - Docker Runner"
echo "================================================"
echo ""

# Check if Docker container is running
if ! docker ps | grep -q "yahboomtechnology/ros-humble"; then
    echo "⚠️  ROS2 Docker container is not running!"
    echo "   Please start it first with: ~/ros2_humble.sh"
    echo ""
    read -p "   Start it now? (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Starting ROS2 container..."
        ~/ros2_humble.sh &
        sleep 5
    else
        exit 1
    fi
fi

# Get container ID
CONTAINER_ID=$(docker ps | grep "yahboomtechnology/ros-humble" | awk '{print $1}')

if [ -z "$CONTAINER_ID" ]; then
    echo "❌ Could not find running ROS2 container"
    exit 1
fi

echo "✓ Found ROS2 container: $CONTAINER_ID"
echo ""

# Copy project files into container
echo "📦 Copying project files into container..."
docker cp ~/MicroROS-Pi5-gesture-control $CONTAINER_ID:/root/

# Install dependencies in container
echo "📥 Installing dependencies..."
docker exec -it $CONTAINER_ID bash -c "cd /root/MicroROS-Pi5-gesture-control && pip3 install opencv-python mediapipe numpy pyyaml"

echo ""
echo "🚀 Starting gesture control..."
echo "   Press Ctrl+C to stop"
echo "   Press 'q' in window to quit"
echo ""

# Run gesture control
docker exec -it $CONTAINER_ID bash -c "cd /root/MicroROS-Pi5-gesture-control && source /opt/ros/humble/setup.bash && python3 main.py"

echo ""
echo "👋 Gesture control stopped."
