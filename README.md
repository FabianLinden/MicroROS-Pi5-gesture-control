# MicroROS-Pi5 Hand Gesture Control System

A comprehensive hand gesture control system for the MicroROS-Pi5 robot using MediaPipe (static gestures) and OpenCV (dynamic gesture tracking).

## Features

- **Dual Detection System**: MediaPipe for static hand poses, OpenCV for dynamic motion tracking
- **Multiple Control Modes**: Movement, Precision, and Gimbal control modes
- **Rich Gesture Set**: 15+ gestures for complete robot control
- **Safety First**: Emergency stop gesture always active, automatic timeout protection
- **ROS2 Integration**: Compatible with MicroROS-Pi5 ESP32 control board
- **Visual Feedback**: Real-time visualization of detected gestures and commands
- **Configurable**: Easy customization through YAML configuration

## Hardware Requirements

- MicroROS-Pi5 robot (Raspberry Pi 5 based)
- 2MP camera (already installed on robot)
- ESP32 Micro ROS control board
- 370 encoder motors
- 2DOF gimbal for camera

## Software Requirements

- Python 3.10+
- ROS2 Humble
- UV package manager
- OpenCV
- MediaPipe

## Installation

### 1. Install UV (if not already installed)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 2. Clone and Setup Project

```bash
cd ~/repos/MicroROS-Pi5
uv sync
```

This will automatically install all dependencies from `pyproject.toml`.

### 3. Test Camera 📷

```bash
uv run python main.py --test-camera
```

### 4. Test Gesture Recognition 🖐️

```bash
uv run python main.py --test-gestures
```

### 5. Run Full System with Robot 🤖

```bash
# Make sure ROS2 Humble is sourced
source /opt/ros/humble/setup.bash

# Run the gesture control node
uv run python main.py
```

## 🎯 Gesture Reference Guide

### 🖐️ Static Gestures (MediaPipe Detection)

Static gestures are hand shapes/poses that are held steady. MediaPipe detects these instantly.

| Emoji | Gesture | Hand Shape | Function | Active Mode |
|-------|---------|-----------|----------|-------------|
| 🛑 | **Open Palm** | All 5 fingers extended, hand flat | **EMERGENCY STOP** | All Modes |
| ✊ | **Fist** | All fingers folded into palm | Hold Position | All Modes |
| 👍 | **Thumbs Up** | Thumb pointing upward, other fingers folded | Move Forward | Movement |
| 👎 | **Thumbs Down** | Thumb pointing downward, other fingers folded | Move Backward | Movement |
| ✌️ | **Peace Sign** | Index + middle finger extended in V shape | Switch to Gimbal Mode | All Modes |
| ☝️ | **Pointing** | Index finger extended, others folded | Switch to Precision Mode | All Modes |
| 👌 | **OK Sign** | Thumb + index fingertip touching, circle shape | Center Gimbal | Gimbal Mode |
| 1️⃣ | **One Finger** | One finger extended | Set Speed Level 1 (20%) | Movement |
| 2️⃣ | **Two Fingers** | Two fingers extended | Set Speed Level 2 (40%) | Movement |
| 3️⃣ | **Three Fingers** | Three fingers extended | Set Speed Level 3 (60%) | Movement |
| 4️⃣ | **Four Fingers** | Four fingers extended | Set Speed Level 4 (80%) | Movement |
| 5️⃣ | **Five Fingers** | All five fingers extended (same as Open Palm) | Set Speed Level 5 (100%) | Movement |

### 🔄 Dynamic Gestures (OpenCV Motion Tracking)

Dynamic gestures require hand movement over time. OpenCV tracks your hand motion to detect these.

| Emoji | Gesture | Motion Pattern | Function | Active Mode |
|-------|---------|----------------|----------|-------------|
| ⬅️ | **Swipe Left** | Hand moves horizontally left (→ ←) | Turn Left | Movement/Precision |
| ➡️ | **Swipe Right** | Hand moves horizontally right (← →) | Turn Right | Movement/Precision |
| ⬆️ | **Swipe Up** | Hand moves vertically upward (↓ ↑) | Increase Speed / Gimbal Tilt Up | Movement/Gimbal |
| ⬇️ | **Swipe Down** | Hand moves vertically downward (↑ ↓) | Decrease Speed / Gimbal Tilt Down | Movement/Gimbal |
| 🔃 | **Circle Clockwise** | Hand draws circle clockwise (↻) | Rotate Robot Clockwise | Movement |
| 🔄 | **Circle Counter-CW** | Hand draws circle counter-clockwise (↺) | Rotate Robot Counter-CW | Movement |
| 👋 | **Wave** | Repeated left-right motion (↔️↔️) | Switch to Movement Mode | All Modes |
| 🫴 | **Push Forward** | Hand moves toward screen center | Arm Extend (Future) | Arm Mode |
| 🤏 | **Pull Backward** | Hand moves away from screen center | Arm Retract (Future) | Arm Mode |
| 🤌 | **Pinch** | Fingers come together | Gripper Close (Future) | Gripper Mode |
| 🖖 | **Spread** | Fingers spread apart | Gripper Open (Future) | Gripper Mode |

### 🎮 Quick Reference Card

```
╔════════════════════════════════════════════════════════════════╗
║                    GESTURE CONTROL MODES                       ║
╠════════════════════════════════════════════════════════════════╣
║                                                                ║
║  🚨 SAFETY (Always Active)                                     ║
║  🛑 Open Palm     → EMERGENCY STOP                             ║
║  ✊ Fist          → Hold Position                              ║
║                                                                ║
║  🚗 MOVEMENT MODE (Default)                                    ║
║  👍 Thumbs Up     → Forward        ⬆️ Swipe Up    → Speed++    ║
║  👎 Thumbs Down   → Backward       ⬇️ Swipe Down  → Speed--    ║
║  ⬅️ Swipe Left    → Turn Left      🔃 Circle CW   → Rotate CW  ║
║  ➡️ Swipe Right   → Turn Right     🔄 Circle CCW  → Rotate CCW ║
║  1️⃣-5️⃣ Fingers    → Speed Levels 1-5                           ║
║                                                                ║
║  🎯 PRECISION MODE (Slow & Accurate)                           ║
║  ☝️ Pointing      → Enter Mode                                 ║
║  👍 Thumbs Up     → Slow Forward   ⬅️ Swipe Left  → Slow Left  ║
║  👎 Thumbs Down   → Slow Backward  ➡️ Swipe Right → Slow Right ║
║  👋 Wave          → Exit to Movement Mode                      ║
║                                                                ║
║  📹 GIMBAL MODE (Camera Control)                               ║
║  ✌️ Peace Sign    → Enter Mode                                 ║
║  ⬅️ Swipe Left    → Pan Left       ⬆️ Swipe Up    → Tilt Up    ║
║  ➡️ Swipe Right   → Pan Right      ⬇️ Swipe Down  → Tilt Down  ║
║  👌 OK Sign       → Center Camera                              ║
║  👋 Wave          → Exit to Movement Mode                      ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
```

## Control Modes

### 1. Movement Mode (Default)
- Basic robot navigation
- Forward, backward, turning
- Speed adjustment
- Rotation in place

### 2. Precision Mode
- Slow, precise movements
- Same controls as Movement mode but at 50% speed
- Ideal for tight spaces

### 3. Gimbal Mode
- Control camera pan/tilt
- Swipe gestures control gimbal position
- OK sign centers gimbal

## Configuration

Edit `config/gesture_config.yaml` to customize:

```yaml
# Adjust detection sensitivity
min_detection_confidence: 0.7
min_tracking_confidence: 0.5

# Adjust robot speed limits
robot_limits:
  max_linear_speed: 0.5  # m/s
  max_angular_speed: 1.0  # rad/s

# Adjust safety timeout
safety_timeout: 2.0  # seconds

# Customize ROS2 topics
ros_topics:
  cmd_vel: "/cmd_vel"
  servo_control: "/servo_control"
```

## Usage Examples

### Basic Operation

```bash
# Run with visualization
uv run python main.py

# Run without visualization (headless)
uv run python main.py --no-viz

# Use custom config
uv run python main.py --config my_custom_config.yaml

# Use external camera
uv run python main.py --camera 1
```

### Testing & Development

```bash
# Test camera only
uv run python tests/test_camera.py

# Test gesture recognition
uv run python tests/test_gestures.py

# Run specific tests
uv run pytest tests/
```

## Troubleshooting

### Camera Not Detected

```bash
# List available cameras
ls /dev/video*

# Test camera manually
uv run python main.py --test-camera
```

### Poor Gesture Recognition

- Ensure good lighting conditions
- Keep hand clearly visible in frame
- Adjust `min_detection_confidence` in config
- Check camera focus

### ROS2 Connection Issues

```bash
# Check ROS2 topics
ros2 topic list

# Monitor cmd_vel
ros2 topic echo /cmd_vel

# Check node status
ros2 node list
```

### Performance Issues

- Reduce camera resolution in config
- Lower FPS setting
- Disable visualization with `--no-viz`
- Adjust `gesture_history_size` (lower = faster but less accurate)

## Development

### Project Structure

```
MicroROS-Pi5/
├── gesture_control/          # Core gesture recognition modules
│   ├── static_gestures.py    # MediaPipe-based detection
│   ├── dynamic_gestures.py   # OpenCV-based tracking
│   ├── gesture_mapper.py     # Command mapping
│   └── gesture_recognizer.py # Main coordinator
├── ros2_nodes/               # ROS2 integration
│   └── gesture_control_node.py
├── config/                   # Configuration files
├── utils/                    # Visualization utilities
├── tests/                    # Test scripts
└── main.py                   # Entry point
```

### Adding New Gestures

1. **Static Gesture**: Add detection logic to `gesture_control/static_gestures.py`
2. **Dynamic Gesture**: Add detection logic to `gesture_control/dynamic_gestures.py`
3. **Map Command**: Update mappings in `gesture_control/gesture_mapper.py`
4. **Execute**: Add command handling in `ros2_nodes/gesture_control_node.py`

### Code Style

```bash
# Format code
uv run black .

# Check linting
uv run flake8 .

# Run tests
uv run pytest tests/
```

## Safety Notes

⚠️ **Important Safety Information**

- Always test in a safe, open area first
- Keep emergency stop (open palm) gesture ready
- The system will automatically stop if no gesture is detected for 2 seconds
- Never operate near stairs, ledges, or obstacles without supervision
- Test thoroughly before autonomous operation

## Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Test thoroughly
4. Submit a pull request

## License

MIT License - See LICENSE file for details

## Support

For issues specific to MicroROS-Pi5 robot:
- Email: support@yahboom.com
- GitHub Issues: [Yahboom MicroROS-Car-Pi5](https://github.com/YahboomTechnology/MicroROS-Car-Pi5)

For gesture control system issues:
- Create an issue in this repository

## Acknowledgments

- [Yahboom Technology](https://www.yahboom.com) for the MicroROS-Pi5 platform
- [MediaPipe](https://mediapipe.dev/) for hand tracking
- [OpenCV](https://opencv.org/) for computer vision
- [ROS2](https://docs.ros.org/en/humble/) for robotics middleware

## 📋 Printable Gesture Cheat Sheet

Print this out and keep it near your robot for quick reference!

```
╔══════════════════════════════════════════════════════════════════╗
║          MICROROS-PI5 GESTURE CONTROL CHEAT SHEET               ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  🚨 EMERGENCY CONTROLS (Work Anytime!)                           ║
║  ┌──────────────────────────────────────────────────────────┐   ║
║  │ 🛑 OPEN PALM    → Emergency Stop (Highest Priority!)    │   ║
║  │ ✊ FIST          → Hold Position & Stay Still             │   ║
║  └──────────────────────────────────────────────────────────┘   ║
║                                                                  ║
║  🚗 BASIC MOVEMENT (Default Mode)                                ║
║  ┌──────────────────────────────────────────────────────────┐   ║
║  │ FORWARD/BACK:              TURNING:                      │   ║
║  │ 👍 Thumbs Up   → Forward    ⬅️ Swipe Left  → Turn Left   │   ║
║  │ 👎 Thumbs Down → Backward   ➡️ Swipe Right → Turn Right  │   ║
║  │                                                           │   ║
║  │ ROTATION:                  SPEED CONTROL:                │   ║
║  │ 🔃 Circle CW   → Spin Right ⬆️ Swipe Up   → Faster      │   ║
║  │ 🔄 Circle CCW  → Spin Left  ⬇️ Swipe Down → Slower      │   ║
║  │                                                           │   ║
║  │ SPEED LEVELS (Show Fingers):                             │   ║
║  │ 1️⃣ One    → 20%     3️⃣ Three → 60% (Default)             │   ║
║  │ 2️⃣ Two    → 40%     4️⃣ Four  → 80%                       │   ║
║  │ 5️⃣ Five   → 100% (Max Speed!)                            │   ║
║  └──────────────────────────────────────────────────────────┘   ║
║                                                                  ║
║  🎯 PRECISION MODE (Slow & Careful)                              ║
║  ┌──────────────────────────────────────────────────────────┐   ║
║  │ ☝️ POINTING     → Enter Precision Mode                   │   ║
║  │ 👍 Thumbs Up    → Slow Forward                            │   ║
║  │ 👎 Thumbs Down  → Slow Backward                           │   ║
║  │ ⬅️ Swipe Left   → Slow Turn Left                         │   ║
║  │ ➡️ Swipe Right  → Slow Turn Right                        │   ║
║  │ 👋 WAVE         → Exit Precision Mode                     │   ║
║  └──────────────────────────────────────────────────────────┘   ║
║                                                                  ║
║  📹 GIMBAL MODE (Camera Control)                                 ║
║  ┌──────────────────────────────────────────────────────────┐   ║
║  │ ✌️ PEACE SIGN   → Enter Gimbal Mode                      │   ║
║  │ ⬅️ Swipe Left   → Pan Camera Left                        │   ║
║  │ ➡️ Swipe Right  → Pan Camera Right                       │   ║
║  │ ⬆️ Swipe Up     → Tilt Camera Up                         │   ║
║  │ ⬇️ Swipe Down   → Tilt Camera Down                       │   ║
║  │ 👌 OK SIGN      → Center Camera                           │   ║
║  │ 👋 WAVE         → Exit Gimbal Mode                        │   ║
║  └──────────────────────────────────────────────────────────┘   ║
║                                                                  ║
║  💡 PRO TIPS:                                                    ║
║  • Keep hand clearly visible in camera view                     ║
║  • Make gestures deliberately (not too fast)                    ║
║  • Emergency stop (🛑) works instantly from any mode            ║
║  • If robot doesn't respond in 2 seconds, it auto-stops         ║
║  • Use good lighting for best detection                         ║
║  • Start with lower speeds until comfortable!                   ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝

            Powered by MediaPipe 🤖 + OpenCV 👁️ + ROS2 ⚙️
```

## 🎥 Gesture Demo Tips

### Best Practices for Gesture Recognition

1. **🌞 Lighting**: Use good, even lighting - avoid backlighting or shadows
2. **📏 Distance**: Keep hand 1-3 feet from camera for best results
3. **🎯 Position**: Center your hand in the camera frame
4. **⏱️ Timing**: Hold static gestures for ~0.5 seconds
5. **🔄 Movement**: Make dynamic gestures smooth and deliberate
6. **🖐️ Clarity**: Keep hand fully visible, avoid occlusion

### Common Mistakes to Avoid

❌ Moving hand too fast during static gestures  
❌ Making gestures outside camera view  
❌ Poor lighting making hand hard to detect  
❌ Fingers not clearly extended/folded  
❌ Starting robot control without testing first  

✅ Test with `--test-gestures` first  
✅ Practice gestures until detection is reliable  
✅ Start with low speeds (level 1-2)  
✅ Keep emergency stop (open palm) ready  
✅ Ensure camera is stable and well-positioned  

## References

- [MicroROS-Pi5 Documentation](https://www.yahboom.net/study/MicroROS-Pi5)
- [MicroROS-Pi5 GitHub](https://github.com/YahboomTechnology/MicroROS-Car-Pi5)
- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html)

