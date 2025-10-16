# 🚀 Quick Start Guide - New Features

This guide helps you quickly get started with the new improvements in v1.1.0.

---

## 📊 Confidence Scores

### What It Does
Shows how confident the system is about detected gestures (0.0 = not confident, 1.0 = very confident).

### How to Use

**See Confidence in Visualization**:
```
Static: thumbs_up (0.90)  ← 90% confident
Dynamic: swipe_left (0.85) ← 85% confident
```

**Adjust Sensitivity**:
```yaml
# config/gesture_config.yaml
min_confidence_threshold: 0.7  # Lower = more sensitive, Higher = more strict
```

**Recommended Settings**:
- **Good lighting**: 0.7 (default)
- **Poor lighting**: 0.6
- **Reduce false positives**: 0.8

---

## 🎯 Gesture Smoothing

### What It Does
Requires gestures to be detected consistently over multiple frames before accepting them.

### How to Use

**Adjust Smoothing Window**:
```yaml
# config/gesture_config.yaml
gesture_smoothing_window: 3  # Number of frames (1-10)
```

**Recommended Settings**:
- **Fast response**: 2 frames
- **Balanced** (default): 3 frames
- **Very stable**: 4-5 frames

**Trade-offs**:
- Smaller window = faster response, more false positives
- Larger window = slower response, fewer false positives

---

## 📝 Enhanced Logging

### What It Does
Provides detailed logs for debugging and monitoring.

### How to Use

**Set Log Level**:
```yaml
# config/gesture_config.yaml
log_level: "INFO"  # DEBUG, INFO, WARN, ERROR, CRITICAL
```

**Log Levels Explained**:

**DEBUG** - Everything (use for troubleshooting):
```
[DEBUG] Command: move_forward | Static: 0.90 | Dynamic: 0.00
[DEBUG] Gesture smoothing: 3/3 votes for thumbs_up
```

**INFO** - Important events (default):
```
[INFO] Speed adjusted to level 3
[INFO] Control mode changed to: gimbal
```

**WARN** - Warnings and safety events:
```
[WARN] EMERGENCY STOP activated!
[WARN] Safety timeout (2.0s) - robot stopped
```

**ERROR** - Errors only:
```
[ERROR] Failed to capture frame
[ERROR] Camera not accessible
```

**When to Use Each Level**:
- Development: `DEBUG`
- Testing: `INFO`
- Production: `INFO` or `WARN`
- Troubleshooting: `DEBUG`

---

## ✅ Configuration Validation

### What It Does
Checks your configuration file for errors before starting the system.

### How to Use

**Automatic Validation** (runs on startup):
```bash
uv run python main.py
```

Output:
```
Validating configuration...
✓ Configuration is valid
```

**Manual Validation**:
```bash
uv run python utils/config_validator.py config/gesture_config.yaml
```

**Common Errors**:

❌ **Missing Required Field**:
```
- Missing required field: camera_width
```
Fix: Add the missing field to your config

❌ **Wrong Type**:
```
- Invalid type for fps: expected int, got str
```
Fix: Change `fps: "30"` to `fps: 30`

❌ **Out of Range**:
```
- Value for min_detection_confidence (1.5) is above maximum (1.0)
```
Fix: Use a value between 0.0 and 1.0

---

## 🧪 Unit Tests

### What It Does
Verifies that the gesture detection system works correctly.

### How to Use

**Run All Tests**:
```bash
uv run pytest tests/test_unit_gestures.py -v
```

**Run Specific Test**:
```bash
# Test gesture mapper
uv run pytest tests/test_unit_gestures.py::TestGestureMapper -v

# Test static gestures
uv run pytest tests/test_unit_gestures.py::TestStaticGestureDetector -v

# Test config validation
uv run pytest tests/test_unit_gestures.py::TestConfigValidator -v
```

**Expected Output**:
```
tests/test_unit_gestures.py::TestGestureMapper::test_safety_gesture_priority PASSED
tests/test_unit_gestures.py::TestGestureMapper::test_mode_switching PASSED
...
========================= 25 passed in 2.34s =========================
```

**When to Run Tests**:
- After updating code
- Before deploying to robot
- When troubleshooting issues
- After changing configuration

---

## 🎛️ Quick Configuration Examples

### Example 1: Maximum Reliability (Fewer False Positives)

```yaml
# config/gesture_config.yaml
min_confidence_threshold: 0.8
gesture_smoothing_window: 4
gesture_cooldown: 0.7
log_level: "INFO"
```

**Best for**: Production use, crowded environments

### Example 2: Maximum Responsiveness (Faster Response)

```yaml
# config/gesture_config.yaml
min_confidence_threshold: 0.6
gesture_smoothing_window: 2
gesture_cooldown: 0.3
log_level: "INFO"
```

**Best for**: Demos, good lighting conditions

### Example 3: Debugging Setup

```yaml
# config/gesture_config.yaml
min_confidence_threshold: 0.7
gesture_smoothing_window: 3
gesture_cooldown: 0.5
log_level: "DEBUG"
```

**Best for**: Development, troubleshooting

### Example 4: Poor Lighting Conditions

```yaml
# config/gesture_config.yaml
min_detection_confidence: 0.6  # Lower MediaPipe threshold
min_tracking_confidence: 0.4
min_confidence_threshold: 0.6  # Lower acceptance threshold
gesture_smoothing_window: 4    # More smoothing to reduce noise
log_level: "INFO"
```

**Best for**: Dim environments, outdoor use

---

## 🔧 Troubleshooting with New Features

### Problem: Gestures Not Detected

**Check Confidence Scores**:
1. Set log level to DEBUG
2. Watch the visualization for confidence values
3. If confidence is low (<0.7), improve lighting or hand position

**Adjust Settings**:
```yaml
min_confidence_threshold: 0.6  # Lower threshold
gesture_smoothing_window: 2    # Reduce smoothing
```

### Problem: Too Many False Positives

**Increase Strictness**:
```yaml
min_confidence_threshold: 0.8  # Higher threshold
gesture_smoothing_window: 4    # More smoothing
gesture_cooldown: 0.7          # Longer cooldown
```

### Problem: Slow Response

**Reduce Smoothing**:
```yaml
gesture_smoothing_window: 2    # Fewer frames required
```

### Problem: Configuration Errors

**Validate Config**:
```bash
uv run python utils/config_validator.py config/gesture_config.yaml
```

**Check for**:
- Missing required fields
- Wrong data types (string vs number)
- Values out of range
- Invalid log level

---

## 📈 Performance Tips

### Optimize for Speed

```yaml
# Reduce processing overhead
camera_width: 320
camera_height: 240
fps: 20
gesture_history_size: 20
log_level: "WARN"  # Minimal logging
```

### Optimize for Accuracy

```yaml
# Better detection quality
camera_width: 640
camera_height: 480
fps: 30
gesture_history_size: 30
min_confidence_threshold: 0.8
gesture_smoothing_window: 4
log_level: "INFO"
```

---

## 🎯 Best Practices

### 1. Start with Defaults
```yaml
min_confidence_threshold: 0.7
gesture_smoothing_window: 3
log_level: "INFO"
```

### 2. Tune Based on Environment
- Good lighting → Keep defaults
- Poor lighting → Lower thresholds
- Noisy environment → Increase smoothing

### 3. Use DEBUG for Development
```yaml
log_level: "DEBUG"
```
Then switch to INFO for production.

### 4. Validate Before Deploying
```bash
uv run python utils/config_validator.py config/gesture_config.yaml
uv run pytest tests/test_unit_gestures.py -v
```

### 5. Monitor Confidence Scores
Watch the visualization to see if gestures are being detected with good confidence.

---

## 🚀 Quick Commands Reference

```bash
# Validate configuration
uv run python utils/config_validator.py config/gesture_config.yaml

# Run unit tests
uv run pytest tests/test_unit_gestures.py -v

# Test with DEBUG logging
# (Edit config: log_level: "DEBUG")
uv run python main.py --test-gestures

# Test camera
uv run python main.py --test-camera

# Run full system
source /opt/ros/humble/setup.bash
uv run python main.py
```

---

## 📚 More Information

- **Detailed Documentation**: See `IMPROVEMENTS.md`
- **Full Changelog**: See `CHANGELOG.md`
- **Main README**: See `README.md`
- **Implementation Guide**: See `REALVNC_IMPLEMENTATION_GUIDE.md`

---

## ✨ Summary

The new improvements make the system:
- ✅ More reliable (confidence scores + smoothing)
- ✅ Easier to debug (enhanced logging)
- ✅ Safer (configuration validation)
- ✅ More maintainable (unit tests)

**Start with the defaults and tune based on your specific needs!**

Happy gesture controlling! 🤖✋
