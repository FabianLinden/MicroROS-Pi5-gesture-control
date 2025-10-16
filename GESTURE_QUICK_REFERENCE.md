# 🤖 MicroROS-Pi5 Gesture Control - Quick Reference Card

**Print this page and keep it near your robot!**

---

## 🚨 EMERGENCY CONTROLS (Always Active)

| Gesture | Action | Priority |
|---------|--------|----------|
| 🛑 **OPEN PALM** | **EMERGENCY STOP** | ⚡ HIGHEST |
| ✊ **FIST** | Hold Position | High |

**These work in ANY mode at ANY time!**

---

## 🚗 MOVEMENT MODE (Default)

### Basic Controls

| Gesture | Function | Description |
|---------|----------|-------------|
| 👍 **Thumbs Up** | Move Forward | Robot moves forward |
| 👎 **Thumbs Down** | Move Backward | Robot moves backward |
| ⬅️ **Swipe Left** | Turn Left | Robot turns left |
| ➡️ **Swipe Right** | Turn Right | Robot turns right |

### Advanced Movement

| Gesture | Function | Description |
|---------|----------|-------------|
| 🔃 **Circle Clockwise** | Rotate Right | Spin in place clockwise |
| 🔄 **Circle Counter-CW** | Rotate Left | Spin in place counter-clockwise |
| ⬆️ **Swipe Up** | Increase Speed | Speed level goes up |
| ⬇️ **Swipe Down** | Decrease Speed | Speed level goes down |

### Speed Levels (Show Fingers)

| Fingers | Speed | Percentage |
|---------|-------|------------|
| 1️⃣ **One** | Level 1 | 20% |
| 2️⃣ **Two** | Level 2 | 40% |
| 3️⃣ **Three** | Level 3 | 60% (Default) |
| 4️⃣ **Four** | Level 4 | 80% |
| 5️⃣ **Five** | Level 5 | 100% (Max) |

---

## 🎯 PRECISION MODE (Slow & Careful)

### Entering/Exiting

| Gesture | Function |
|---------|----------|
| ☝️ **Pointing** | Enter Precision Mode |
| 👋 **Wave** | Exit to Movement Mode |

### Precision Controls (All at 50% Speed)

| Gesture | Function |
|---------|----------|
| 👍 **Thumbs Up** | Slow Forward |
| 👎 **Thumbs Down** | Slow Backward |
| ⬅️ **Swipe Left** | Slow Turn Left |
| ➡️ **Swipe Right** | Slow Turn Right |

**Best for: Tight spaces, delicate maneuvers, testing**

---

## 📹 GIMBAL MODE (Camera Control)

### Entering/Exiting

| Gesture | Function |
|---------|----------|
| ✌️ **Peace Sign** | Enter Gimbal Mode |
| 👋 **Wave** | Exit to Movement Mode |

### Gimbal Controls

| Gesture | Function | Description |
|---------|----------|-------------|
| ⬅️ **Swipe Left** | Pan Left | Camera looks left |
| ➡️ **Swipe Right** | Pan Right | Camera looks right |
| ⬆️ **Swipe Up** | Tilt Up | Camera looks up |
| ⬇️ **Swipe Down** | Tilt Down | Camera looks down |
| 👌 **OK Sign** | Center Camera | Reset to center position |

---

## 💡 PRO TIPS

### For Best Results

✅ **Good Lighting** - Use bright, even lighting  
✅ **Camera Distance** - Keep hand 1-3 metres from camera  
✅ **Center Hand** - Keep hand in middle of camera view  
✅ **Clear Gestures** - Make deliberate, clear hand shapes  
✅ **Hold Static** - Hold static gestures for ~0.5 seconds  
✅ **Smooth Dynamic** - Make swipes and circles smoothly  

### Safety First

🛑 **Always test emergency stop first**  
⏱️ **Robot auto-stops after 2 seconds of no gesture**  
🐢 **Start with low speeds (Level 1-2)**  
👁️ **Keep visual contact with robot**  
🚫 **Never operate near stairs or ledges**  
✋ **Keep emergency stop gesture ready**  

### Common Mistakes to Avoid

❌ Moving hand too fast  
❌ Making gestures outside camera view  
❌ Poor lighting conditions  
❌ Fingers not clearly extended/folded  
❌ Starting at high speeds  

### Troubleshooting

| Problem | Solution |
|---------|----------|
| Gestures not detected | Check lighting, center hand in view |
| Wrong gesture detected | Make clearer hand shapes, hold longer |
| Robot not responding | Check ROS2 connection, verify topics |
| Slow detection | Reduce camera resolution in config |
| Robot keeps stopping | Make gestures more frequently |

---

## 📊 Control Mode Summary

```
┌─────────────┬──────────────────┬────────────────────────┐
│ Mode        │ Enter With       │ Exit With              │
├─────────────┼──────────────────┼────────────────────────┤
│ Movement    │ Default / 👋 Wave│ ☝️ Point / ✌️ Peace   │
│ Precision   │ ☝️ Pointing      │ 👋 Wave                │
│ Gimbal      │ ✌️ Peace Sign    │ 👋 Wave                │
└─────────────┴──────────────────┴────────────────────────┘
```

---

## 🔧 Quick Commands

```bash
# Test camera
uv run python main.py --test-camera

# Test gestures (no robot)
uv run python main.py --test-gestures

# Run full system
source /opt/ros/humble/setup.bash
uv run python main.py

# Run without visualization
uv run python main.py --no-viz
```

---

**Powered by MediaPipe 🤖 + OpenCV 👁️ + ROS2 ⚙️**

© 2025 MicroROS-Pi5 Gesture Control System

---

**Print this reference card and laminate it for durability!**

