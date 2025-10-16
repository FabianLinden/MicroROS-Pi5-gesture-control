# 🖥️ Complete Implementation Guide for MicroROS-Pi5 with RealVNC

## 📋 Prerequisites Checklist

Before starting, ensure you have:
- ✅ MicroROS-Pi5 robot powered on
- ✅ RealVNC Server running on Raspberry Pi 5
- ✅ RealVNC Viewer installed on your Windows PC
- ✅ Both devices on the same network
- ✅ Pi5 IP address (check with `hostname -I` on Pi5)
- ✅ Camera connected and working on Pi5

---

## 🚀 Step-by-Step Implementation

### **Step 1: Connect to Your Pi5 via RealVNC**

1. **Open RealVNC Viewer on Windows**
   ```
   - Launch RealVNC Viewer
   - Enter your Pi5 IP address (e.g., 192.168.1.100)
   - Click "Connect"
   - Enter username (usually "pi") and password
   ```

2. **Verify Connection**
   ```
   - You should see the Pi5 desktop
   - Test that mouse/keyboard work
   - Open a terminal to verify
   ```

---

### **Step 2: Transfer Project Files to Pi5**

**Option A: Using RealVNC File Transfer (Easiest)**

1. **In RealVNC Viewer window:**
   ```
   - Click the toolbar icon for "File Transfer"
   - Navigate to your Windows project folder:
     C:\Users\FabianLinden\repos\MicroROS-Pi5
   - Select all project files
   - In the right pane (Pi5), navigate to: /home/pi/repos/
   - Click "Send" to transfer files
   ```

**Option B: Using Git (Recommended for updates)**

1. **On Pi5 terminal (via VNC):**
   ```bash
   cd ~
   mkdir -p repos
   cd repos
   
   # If you have a git repo, clone it:
   # git clone <your-repo-url> MicroROS-Pi5
   
   # Otherwise, create the directory
   mkdir -p MicroROS-Pi5
   ```

2. **Then use RealVNC File Transfer to copy files into this folder**

**Option C: Using SCP from Windows PowerShell**

1. **On your Windows PC (PowerShell):**
   ```powershell
   # Replace <PI_IP> with your Pi5's IP address
   scp -r C:\Users\FabianLinden\repos\MicroROS-Pi5 pi@<PI_IP>:~/repos/
   ```

---

### **Step 3: Install UV Package Manager on Pi5**

1. **In VNC terminal on Pi5:**
   ```bash
   # Install UV
   curl -LsSf https://astral.sh/uv/install.sh | sh
   
   # Add UV to PATH for current session
   source $HOME/.cargo/env
   
   # Make it permanent (add to .bashrc)
   echo 'source $HOME/.cargo/env' >> ~/.bashrc
   
   # Verify installation
   uv --version
   ```

   Expected output: `uv x.x.x`

---

### **Step 4: Setup Project Dependencies**

1. **Navigate to project:**
   ```bash
   cd ~/repos/MicroROS-Pi5
   
   # Verify files are there
   ls -la
   ```

   You should see:
   ```
   gesture_control/
   ros2_nodes/
   utils/
   tests/
   config/
   main.py
   pyproject.toml
   README.md
   ...
   ```

2. **Install Python dependencies:**
   ```bash
   # Sync dependencies with UV
   uv sync
   ```

   This installs:
   - opencv-python
   - mediapipe
   - numpy
   - pyyaml

   Wait for installation to complete (~2-5 minutes)

---

### **Step 5: Test Camera**

1. **Check camera is connected:**
   ```bash
   # List video devices
   ls -l /dev/video*
   ```

   You should see: `/dev/video0` (or similar)

2. **Test camera with gesture control:**
   ```bash
   uv run python main.py --test-camera
   ```

3. **What you should see:**
   - OpenCV window opens showing camera feed
   - "Camera Test - Press 'q' to quit" text overlay
   - Live video from your camera

4. **Troubleshooting:**
   ```bash
   # If camera not found, check permissions
   sudo usermod -a -G video $USER
   
   # Reboot if needed
   sudo reboot
   
   # Reconnect VNC and try again
   ```

5. **Press 'q' to close camera test** - **Hover over the camera screen and press q**

---

### **Step 6: Test Gesture Recognition (No Robot)**

1. **Test gesture detection safely:**
   ```bash
   cd ~/repos/MicroROS-Pi5
   uv run python main.py --test-gestures
   ```

2. **What you should see:**
   - OpenCV window with camera feed
   - Your hand with landmarks drawn on it (green lines)
   - Detected gestures shown on screen
   - Current mode and speed level displayed

3. **Try these gestures:**
   ```
   🖐️ Open Palm    → Should show "Static: open_palm"
   ✊ Fist          → Should show "Static: fist"
   👍 Thumbs Up     → Should show "Static: thumbs_up"
   👎 Thumbs Down   → Should show "Static: thumbs_down"
   ✌️ Peace Sign    → Should show "Static: peace_sign"
   ⬅️ Swipe Left    → Should show "Dynamic: swipe_left"
   ➡️ Swipe Right   → Should show "Dynamic: swipe_right"
   ```

4. **Tips for good detection:**
   - Keep hand 1-3 metres from camera
   - Use good lighting
   - Center hand in camera view
   - Hold static gestures for 0.5 seconds
   - Make dynamic gestures smoothly

5. **Watch terminal output:**
   ```
   Command: move_forward          | Mode: movement      | Speed: 3
   Command: turn_left             | Mode: movement      | Speed: 3
   Command: mode_switch_gimbal    | Mode: gimbal        | Speed: 3
   ```

6. **Press 'q' to exit**

---

### **Step 7: Check ROS2 Setup**

1. **Source ROS2 Humble:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Add to .bashrc for automatic sourcing:**
   ```bash
   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
   ```

3. **Verify ROS2 is working:**
   ```bash
   # Check ROS2 environment
   printenv | grep ROS
   
   # Should show:
   # ROS_VERSION=2
   # ROS_DISTRO=humble
   # ROS_PYTHON_VERSION=3
   ```

4. **Check if robot nodes are running:**
   ```bash
   # List active ROS2 nodes
   ros2 node list
   ```

   You might see nodes like:
   ```
   /micro_ros_agent
   /robot_base_controller
   /lidar_node
   ```

5. **Check available topics:**
   ```bash
   # List all topics
   ros2 topic list
   
   # Look for:
   /cmd_vel           # Movement commands
   /servo_control     # Servo/gimbal control
   ```

6. **Test cmd_vel topic:**
   ```bash
   # Check topic info
   ros2 topic info /cmd_vel
   
   # Should show:
   # Type: geometry_msgs/msg/Twist
   # Publisher count: X
   # Subscription count: Y
   ```

---

### **Step 8: Configure Topics (If Needed)**

1. **If your robot uses different topic names:**
   ```bash
   # Open config file
   nano ~/repos/MicroROS-Pi5/config/gesture_config.yaml
   ```

2. **Update topic names:**
   ```yaml
   # ROS2 Topics
   ros_topics:
     cmd_vel: "/cmd_vel"              # Change if different
     servo_control: "/servo_control"  # Change if different
     gesture_status: "/gesture_status"
   ```

3. **Save and exit:**
   ```
   Ctrl+O (save)
   Enter (confirm)
   Ctrl+X (exit)
   ```

---

### **Step 9: Adjust Robot Speed Limits (Safety)**

1. **Edit configuration for safe initial speeds:**
   ```bash
   nano ~/repos/MicroROS-Pi5/config/gesture_config.yaml
   ```

2. **Set conservative speed limits:**
   ```yaml
   # Robot Limits - Start slow!
   robot_limits:
     max_linear_speed: 0.3   # m/s (reduced from 0.5)
     max_angular_speed: 0.5  # rad/s (reduced from 1.0)
     gimbal_step: 10.0       # degrees
   
   # Safety Settings
   safety_timeout: 2.0  # seconds - auto-stop if no gesture
   ```

3. **Save and exit (Ctrl+O, Enter, Ctrl+X)**

---

### **Step 10: Run Full Gesture Control System**

1. **Make sure you're in a safe test area!**
   ```
   ⚠️ SAFETY CHECKLIST:
   - Robot on flat surface
   - Clear space around robot (2m minimum)
   - No stairs or ledges nearby
   - Emergency stop (open palm) ready
   - Someone watching the robot
   ```

2. **Start the gesture control system:**
   ```bash
   cd ~/repos/MicroROS-Pi5
   source /opt/ros/humble/setup.bash
   uv run python main.py
   ```

3. **What happens:**
   ```
   Starting MicroROS-Pi5 Gesture Control System...
   Config: config/gesture_config.yaml
   Camera: 0
   ✓ ROS2 Humble sourced
   ✓ Starting gesture control...
   Gesture Control Node initialized
   Control mode: ControlMode.MOVEMENT
   ```

4. **You'll see:**
   - OpenCV window with camera feed
   - Hand tracking landmarks
   - Detected gestures
   - Current command
   - Control mode (Movement/Precision/Gimbal)
   - Speed level bar

---

### **Step 11: Test Robot Control (Carefully!)**

1. **First, test EMERGENCY STOP:**
   ```
   🛑 Show OPEN PALM gesture
   - Robot should stop immediately
   - Terminal shows: "EMERGENCY STOP!"
   ```

2. **Test basic movement at LOW speed:**
   ```
   1️⃣ Show ONE FINGER → Sets speed to Level 1 (20%)
   👍 Thumbs Up        → Robot moves forward slowly
   🛑 Open Palm        → STOP
   ```

3. **Test turning:**
   ```
   1️⃣ Show ONE FINGER → Speed Level 1
   ⬅️ Swipe Left      → Robot turns left slowly
   🛑 Open Palm       → STOP
   ➡️ Swipe Right     → Robot turns right slowly
   🛑 Open Palm       → STOP
   ```

4. **Test other gestures:**
   ```
   👎 Thumbs Down     → Move backward
   🔃 Circle CW       → Rotate clockwise
   🔄 Circle CCW      → Rotate counter-clockwise
   ```

5. **Test mode switching:**
   ```
   ☝️ Pointing        → Switch to PRECISION mode
   👍 Thumbs Up       → Move forward (slower)
   👋 Wave            → Back to MOVEMENT mode
   
   ✌️ Peace Sign      → Switch to GIMBAL mode
   ⬅️ Swipe Left      → Pan camera left
   ⬆️ Swipe Up        → Tilt camera up
   👌 OK Sign         → Center camera
   👋 Wave            → Back to MOVEMENT mode
   ```

---

### **Step 12: Monitor ROS2 Topics (Optional)**

1. **Open a second terminal in VNC:**
   ```bash
   # In new terminal
   source /opt/ros/humble/setup.bash
   
   # Monitor movement commands
   ros2 topic echo /cmd_vel
   ```

2. **You'll see Twist messages when you make gestures:**
   ```yaml
   linear:
     x: 0.3
     y: 0.0
     z: 0.0
   angular:
     x: 0.0
     y: 0.0
     z: 0.0
   ```

3. **Monitor gesture status:**
   ```bash
   ros2 topic echo /gesture_status
   ```

---

### **Step 13: Stopping the System**

1. **To stop gesture control:**
   ```
   - Click on OpenCV window
   - Press 'q' key
   OR
   - In terminal: Ctrl+C
   ```

2. **System will:**
   ```
   - Stop the robot (sends zero velocity)
   - Release camera
   - Close all windows
   - Clean up resources
   ```

---

## 🔧 Optimization for VNC

### **If Video Feed is Laggy:**

1. **Reduce camera resolution:**
   ```bash
   nano ~/repos/MicroROS-Pi5/config/gesture_config.yaml
   ```

   ```yaml
   # Camera Settings
   camera_width: 320   # Reduced from 640
   camera_height: 240  # Reduced from 480
   fps: 20            # Reduced from 30
   ```

2. **Or run without visualization:**
   ```bash
   uv run python main.py --no-viz
   ```

---

## 📱 Creating a Launcher Script

1. **Create easy-to-run script:**
   ```bash
   nano ~/start_gesture_control.sh
   ```

2. **Add this content:**
   ```bash
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
   ```

3. **Make executable:**
   ```bash
   chmod +x ~/start_gesture_control.sh
   ```

4. **Run anytime with:**
   ```bash
   ~/start_gesture_control.sh
   ```

---

## 🎯 Create Desktop Shortcut (VNC)

1. **Create desktop launcher:**
   ```bash
   nano ~/Desktop/GestureControl.desktop
   ```

2. **Add this content:**
   ```ini
   [Desktop Entry]
   Type=Application
   Name=Gesture Control
   Comment=MicroROS-Pi5 Hand Gesture Control
   Exec=/home/pi/start_gesture_control.sh
   Icon=input-gaming
   Terminal=true
   Categories=Application;
   ```

3. **Make executable:**
   ```bash
   chmod +x ~/Desktop/GestureControl.desktop
   ```

4. **Now you can double-click icon on Pi5 desktop to launch!**

---

## 🐛 Troubleshooting Guide

### **Camera Issues**

**Problem:** Camera not detected
```bash
# Solution 1: Check camera
ls -l /dev/video*

# Solution 2: Test with raspistill
raspistill -o test.jpg

# Solution 3: Add user to video group
sudo usermod -a -G video $USER
sudo reboot
```

**Problem:** "Failed to capture frame"
```bash
# Try different camera index
uv run python main.py --camera 1
```

---

### **Gesture Detection Issues**

**Problem:** Gestures not detected
```
Solutions:
1. Improve lighting (add desk lamp)
2. Move closer to camera (1-3 metres)
3. Center hand in camera view
4. Clean camera lens
5. Make clearer hand shapes
```

**Problem:** Wrong gestures detected
```
Solutions:
1. Hold static gestures longer (0.5+ seconds)
2. Make dynamic gestures more deliberately
3. Reduce gesture cooldown in config
4. Adjust confidence thresholds in config
```

---

### **ROS2 Connection Issues**

**Problem:** Robot not responding to gestures
```bash
# Check 1: ROS2 sourced?
printenv | grep ROS_DISTRO

# Check 2: Topics exist?
ros2 topic list | grep cmd_vel

# Check 3: Can we publish manually?
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

# Check 4: Is robot node running?
ros2 node list

# Check 5: Check topic connections
ros2 topic info /cmd_vel
```

**Problem:** "No module named 'rclpy'"
```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Add to .bashrc permanently
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

---

### **Performance Issues**

**Problem:** Slow gesture detection
```yaml
# Edit config/gesture_config.yaml
camera_width: 320      # Lower resolution
camera_height: 240
fps: 15               # Lower FPS
gesture_history_size: 20  # Smaller history
```

**Problem:** VNC video feed laggy
```bash
# Run without visualization
uv run python main.py --no-viz

# Gestures still work, just no video window
```

---

## ✅ Quick Reference Commands

```bash
# Test camera only
uv run python main.py --test-camera

# Test gestures (safe, no robot)
uv run python main.py --test-gestures

# Run full system
source /opt/ros/humble/setup.bash && uv run python main.py

# Run without video window (better performance)
uv run python main.py --no-viz

# Check ROS2 topics
ros2 topic list

# Monitor movement commands
ros2 topic echo /cmd_vel

# Check active nodes
ros2 node list

# Use launcher script
~/start_gesture_control.sh
```

---

## 📊 Expected Behavior Summary

| Stage | Expected Behavior |
|-------|------------------|
| **Camera Test** | Window opens, shows video, press 'q' to quit |
| **Gesture Test** | Hand landmarks appear, gestures detected, commands shown |
| **Full System** | All above + robot moves according to gestures |
| **Emergency Stop** | Open palm → robot stops immediately |
| **No Gesture** | After 2 seconds → robot auto-stops |
| **Mode Switch** | Peace sign → gimbal mode, Wave → movement mode |

---

## 🎓 Learning Path

### **Day 1: Setup & Testing**
1. Install everything (Steps 1-4)
2. Test camera (Step 5)
3. Practice gestures without robot (Step 6)

### **Day 2: Integration**
1. Configure ROS2 (Steps 7-9)
2. Test with robot at lowest speed
3. Practice emergency stop

### **Day 3: Mastery**
1. Try all gestures
2. Test all modes
3. Increase speed gradually
4. Create custom configurations

---

## 🎯 Success Checklist

- [ ] RealVNC connected to Pi5
- [ ] Project files transferred
- [ ] UV installed
- [ ] Dependencies installed (`uv sync`)
- [ ] Camera test working
- [ ] Gestures detected in test mode
- [ ] ROS2 sourced and working
- [ ] Topics configured correctly
- [ ] Emergency stop tested
- [ ] Robot responds to gestures
- [ ] All modes working
- [ ] Launcher script created

---

## 🆘 Getting Help

1. **Check terminal output** for error messages
2. **Review troubleshooting section** above
3. **Check camera/lighting** if detection fails
4. **Verify ROS2 connection** if robot doesn't move
5. **Start with test modes** before full system
6. **Reduce speeds** if robot moves too fast

**Support:**
- Robot: support@yahboom.com
- GitHub: github.com/YahboomTechnology/MicroROS-Car-Pi5

---

**🎉 You're ready to control your robot with hand gestures!**

Remember: Always start with **Speed Level 1**, test **Emergency Stop** first, and keep your hand visible to the camera!

