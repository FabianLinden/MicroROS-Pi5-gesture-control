#!/bin/bash
# Modified ROS2 startup script that includes gesture control project

echo "🤖 Starting ROS2 with Gesture Control Support"
echo "=============================================="

xhost +

docker run -it \
--privileged=true \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /dev/input:/dev/input \
-v /dev/video0:/dev/video0 \
-v ~/MicroROS-Pi5-gesture-control:/root/gesture-control \
yahboomtechnology/ros-humble:4.1.2 \
bash -c "
    echo '✓ ROS2 Container Started'
    echo '✓ Gesture control mounted at /root/gesture-control'
    echo ''
    echo 'Installing Python dependencies...'
    pip3 install opencv-python mediapipe numpy pyyaml
    echo ''
    echo '✓ Dependencies installed'
    echo ''
    echo 'To run gesture control:'
    echo '  cd /root/gesture-control'
    echo '  source /opt/ros/humble/setup.bash'
    echo '  python3 main.py'
    echo ''
    /bin/bash
"
