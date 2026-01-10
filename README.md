# Puzzlebot - Autonomous Line Following with Traffic Sign Detection

## Project Overview

This is a ROS 2 (Robot Operating System 2) package for autonomous navigation of a differential drive robot (Puzzlebot). The system implements **line following with dual PID control**, **traffic light detection**, **traffic sign recognition using YOLO deep learning**, and **hierarchical finite state machine (FSM) control**.

The robot can:
- Follow a line autonomously using computer vision
- Detect and respond to traffic lights (red, yellow, green)
- Recognize traffic signs (stop, straight, left, right)
- Execute complex maneuvers (crossings, turns, stops)
- Handle edge cases like line loss and recovery

---

## System Architecture

### Node Communication Diagram

```
┌──────────────────────┐
│  image_viewer_node   │  ◄─── Camera: /video_source/compressed
│  (Line Following +   │
│   Dual PID Control)  │
└──────────┬───────────┘
           │ publishes
           ├─► /cmd_vel (Twist)           - Movement commands
           └─► /line_lost (Bool)          - Line detection status
                ▲
                │ reads
┌───────────────┴────────────┐
│       master_node          │
│  (Hierarchical FSM)        │
└───────┬────────────────────┘
        │ publishes
        ├─► /cmd_vel (Twist)            - Override commands
        └─► /control_priority (Bool)    - Who has control
        ▲
        │ subscribes to
        ├─── /signal_detected (Int32)   ◄─── ml_yolo node
        ├─── /light_state_int (Int32)   ◄─── traffic_light_node
        └─── /line_lost (Bool)          ◄─── image_viewer_node

┌──────────────────────┐
│     ml_yolo          │  ◄─── Camera: /video_source/compressed
│  (YOLO Traffic Sign  │
│    Detection)        │
└──────────┬───────────┘
           └─► /signal_detected (Int32)
                - 0: None
                - 1: Stop
                - 2: Straight (crossing)
                - 3: Left turn
                - 4: Right turn

┌──────────────────────┐
│  traffic_light_node  │  ◄─── Camera: /video_source/compressed
│  (Traffic Light HSV  │
│    Color Detection)  │
└──────────┬───────────┘
           └─► /light_state_int (Int32)
                - 0: Unknown
                - 1: Red
                - 2: Yellow
                - 3: Green
```

---

## Nodes Description

### 1. **image_viewer_node** (Line Following with Dual PID)

**Purpose:** Primary line tracking and following using computer vision and dual PID control.

**Functionality:**
- Processes compressed camera images at 15 FPS
- Applies image processing pipeline:
  - Gaussian blur and sharpening
  - ROI (Region of Interest) selection (bottom 30% of frame)
  - Canny edge detection + binary thresholding
  - Morphological operations to extract vertical lines
  - Contour analysis to find line centroid
- Computes lateral error from image center
- Implements **dual PID control**:
  - **Angular PID:** Adjusts steering (angular.z) to minimize lateral error
  - **Velocity PID per wheel:** Controls left/right wheel speeds independently for precise tracking
- Publishes line status (`/line_lost`) for master node

**Topics:**
- **Subscribes:**
  - `/video_source/compressed` (CompressedImage) - Camera feed
  - `/VelocityEncL` (Float32) - Left wheel encoder velocity
  - `/VelocityEncR` (Float32) - Right wheel encoder velocity
  - `/control_priority` (Bool) - Whether master node has control
- **Publishes:**
  - `/cmd_vel` (Twist) - Movement commands when in control
  - `/line_lost` (Bool) - True if line not detected

**Tunable Parameters (via OpenCV trackbars):**
- Binary threshold, Canny thresholds
- Minimum contour area
- Morphological operation type
- PID gains (Kp, Ki, Kd) for left/right wheels
- Target velocity (cm/s)

---

### 2. **ml_yolo** (Traffic Sign Detection with YOLO)

**Purpose:** Deep learning-based traffic sign detection and classification.

**Functionality:**
- Uses YOLOv8 model (`best.pt`) for real-time object detection
- Detects 4 classes: `stop`, `straight`, `left`, `right`
- Implements robust filtering:
  - **Minimum area threshold per class** (reduces false positives from distant signs)
  - **Spatial filtering:** Left signs must be on left side, right on right side
  - **Confidence threshold:** Only accepts detections > 0.6
  - **Consistency window:** Requires 5+ consistent detections in rolling 15-frame buffer
  - **Persistence:** Maintains last valid detection until cleared by 8+ frames of no detection
- GPU acceleration support (CUDA if available)

**Topics:**
- **Subscribes:**
  - `/video_source/compressed` (CompressedImage) - Camera feed
- **Publishes:**
  - `/signal_detected` (Int32) - Detected sign ID

**Sign ID Mapping:**
```
0 → None
1 → Stop
2 → Straight (proceed through crossing)
3 → Left turn
4 → Right turn
```

**Key Parameters:**
- `detection_window`: 15 frames (maxlen)
- `required_consistency`: 5 detections
- `no_detection_limit`: 8 frames before clearing
- Minimum areas: stop=100px², straight=400px², left/right=300px²

---

### 3. **traffic_light_node** (Traffic Light Color Detection)

**Purpose:** Computer vision-based traffic light state detection using HSV color space.

**Functionality:**
- Processes camera images to detect red, yellow, and green traffic lights
- HSV color space processing:
  - Gaussian blur for noise reduction
  - CLAHE (Contrast Limited Adaptive Histogram Equalization) on V channel
  - Multiple HSV range masks per color for robustness
- Circle detection using contour analysis:
  - Area filtering (30-300px²)
  - Circularity threshold (>0.8)
  - Brightness scoring
- Temporal consistency with deque (5-frame buffer)
- Interactive HSV debug mode (mouse hover shows HSV values)

**Topics:**
- **Subscribes:**
  - `/video_source/compressed` (CompressedImage) - Camera feed
- **Publishes:**
  - `/light_state` (String) - Color name ("red", "yellow", "green", "unknown")
  - `/light_state_int` (Int32) - Numeric state (0-3)

**Color ID Mapping:**
```
0 → Unknown
1 → Red (stop)
2 → Yellow (caution)
3 → Green (go)
```

---

### 4. **master_node** (Hierarchical Finite State Machine)

**Purpose:** High-level decision-making and behavioral control coordinator.

**Functionality:**
- Implements a **hierarchical FSM** with 5 states:
  1. **SEEK_LINE:** Normal line following (image_viewer has control)
  2. **STOPPED:** Robot halted (master has control)
  3. **TURNING:** Executing right turn maneuver (master has control)
  4. **CROSSING:** Crossing intersection straight (master has control)
  5. **LINE_LOST:** Recovery behavior when line is lost (master has control)

- **Line loss debouncing:** Requires 10/15 frames lost before triggering state change
- **Pending action system:** Stores intended action (CROSSING/TURNING) when sign detected, executes when line is lost
- **Traffic light integration:** Waits for green light before crossing
- **Grace period:** 2-second tolerance for brief line loss without changing state
- **Timeout protection:** 4-second max for crossing actions

**State Transition Logic:**
```
SEEK_LINE:
├─ Stop sign (signal=1) → STOPPED
├─ Line lost + pending TURNING → TURNING
├─ Line lost + pending CROSSING + green light → CROSSING
├─ Line lost + pending action + not green → STOPPED
└─ Line lost > 2s without pending → LINE_LOST

STOPPED:
├─ Pending TURNING → TURNING
├─ Pending CROSSING + green light → CROSSING
└─ Timeout 3s → SEEK_LINE

TURNING (3-phase right turn):
├─ Phase 1: Drive forward 1s
├─ Phase 2: Turn right until line found
└─ Phase 3: Final adjustment → SEEK_LINE

CROSSING:
├─ Line found → SEEK_LINE
├─ Timeout 4s → LINE_LOST
└─ Drive forward

LINE_LOST:
├─ Line found → SEEK_LINE
└─ Explore (forward + rotate)
```

**Topics:**
- **Subscribes:**
  - `/signal_detected` (Int32) - Traffic sign from YOLO
  - `/light_state_int` (Int32) - Traffic light state
  - `/line_lost` (Bool) - Line detection status
- **Publishes:**
  - `/cmd_vel` (Twist) - Override movement commands
  - `/control_priority` (Bool) - True when master controls robot

**Key Parameters:**
- `line_buffer`: 15 frames (maxlen)
- `line_threshold`: 10 detections to confirm line loss
- `action_timeout`: 4 seconds
- Turn speed: 0.18 rad/s
- Forward speed: 0.1-0.15 m/s

---

## Launch File

### **traffic_light_launch.py**

Starts all 4 nodes simultaneously:

```bash
ros2 launch puzzlebot traffic_light_launch.py
```

**Nodes launched:**
1. `image_viewer_node` - Line following
2. `ml_yolo` - Traffic sign detection
3. `traffic_light_node` - Traffic light detection
4. `master_node` - FSM controller

All nodes output to screen for debugging.

---

## Installation & Setup

### Prerequisites

**System Requirements:**
- **ROS 2** (Humble or Foxy recommended)
- **Python 3.8+**
- **Git**

**Python Dependencies:**
```bash
pip install opencv-python numpy torch ultralytics cv_bridge
```

### Clone and Build

```bash
# 1. Clone the repository
git clone <repository-url> puzzlebot_ws
cd puzzlebot_ws

# 2. Ensure YOLO model is present
# Place your trained best.pt model in the workspace root
# puzzlebot_ws/best.pt

# 3. Build the workspace
colcon build --symlink-install

# 4. Source the workspace
source install/setup.bash
```

**Note:** The `build/`, `install/`, and `log/` directories are automatically generated by `colcon build` and should not be committed to version control.

### YOLO Model Setup

The system requires a trained YOLOv8 model file named `best.pt` in the workspace root:

```
puzzlebot_ws/
├── best.pt          ← Place your trained YOLOv8 model here
├── src/
│   └── puzzlebot/
├── build/           ← Generated (ignored by git)
├── install/         ← Generated (ignored by git)
└── log/             ← Generated (ignored by git)
```

If you don't have a trained model, you'll need to train one or obtain `best.pt` from your team.

---

## Running the System

### Option 1: Launch All Nodes

```bash
ros2 launch puzzlebot traffic_light_launch.py
```

### Option 2: Run Nodes Individually (for debugging)

```bash
# Terminal 1: Line following
ros2 run puzzlebot image_viewer_node

# Terminal 2: Traffic signs
ros2 run puzzlebot ml_yolo

# Terminal 3: Traffic lights
ros2 run puzzlebot traffic_light_node

# Terminal 4: Master controller
ros2 run puzzlebot master_node
```

---

## Tuning & Calibration

### Line Following (image_viewer_node)

The node provides real-time tuning via OpenCV trackbars in the "Ajustes" window:

1. **Image Processing:**
   - `Threshold bin` (230): Binary threshold for line extraction
   - `Canny min/max` (140/200): Edge detection sensitivity
   - `Área mínima` (250): Minimum contour size to consider
   - `Morph op` (1): Morphological operation type (0=dilate, 1=erode, 2=close)

2. **PID Tuning:**
   - `Kp_L`, `Ki_L`, `Kd_L`: Left wheel PID gains
   - `Kp_R`, `Ki_R`, `Kd_R`: Right wheel PID gains
   - `Velocidad cm/s` (20): Target velocity setpoint

**Tuning Tips:**
- Start with Kp only, increase until oscillation begins
- Add Ki to eliminate steady-state error
- Add Kd to dampen oscillations
- Ensure left/right gains are similar for balanced tracking

### Traffic Sign Detection (ml_yolo)

Edit hardcoded parameters in [ml_yolo.py](src/puzzlebot/puzzlebot/ml_yolo.py):

```python
# Line 22-24: Detection consistency
self.detection_window = deque(maxlen=15)
self.required_consistency = 5      # Frames needed for confirmation
self.no_detection_limit = 8        # Frames before clearing detection

# Line 31-36: Minimum bounding box areas per class
self.min_area_by_class = {
    'stop': 100,      # Increase to reject distant signs
    'straight': 400,
    'left': 300,
    'right': 300,
}
```

### Traffic Light Detection (traffic_light_node)

Adjust HSV ranges in [traffic_light_node.py](src/puzzlebot/puzzlebot/traffic_light_node.py) (lines 57-76) based on lighting conditions. Use mouse hover in "Camera View" window to inspect HSV values.

### Master FSM (master_node)

Edit behavior parameters in [master_node.py](src/puzzlebot/puzzlebot/master_node.py):

```python
# Line 13-17: Line loss detection
self.line_buffer = deque(maxlen=15)
self.line_threshold = 10           # Detections needed to confirm loss

# Line 22-23: Action timing
self.action_timeout = 4.0          # Max seconds for crossing
```

---

## Troubleshooting

### Line Not Detected
- Check "Combinado final" and "Vertical Only" windows
- Adjust `Threshold bin` and `Canny` parameters
- Verify lighting conditions
- Increase `Área mínima` if too noisy

### Traffic Signs Not Recognized
- Ensure `best.pt` model is in correct location
- Check GPU availability: Look for "Using device: cuda" message
- Verify sign is within minimum area threshold
- Check "Traffic Signals" window for detections
- Reduce `required_consistency` for faster detection

### Traffic Lights Not Detected
- Use HSV debug (hover mouse in "Camera View")
- Adjust HSV ranges for your lighting conditions
- Verify traffic light is within 30-300px² area
- Check color brightness in V channel

### Robot Doesn't Stop at Signs
- Verify master_node is receiving `/signal_detected` messages:
  ```bash
  ros2 topic echo /signal_detected
  ```
- Check FSM state transitions in master_node logs
- Ensure traffic light is green for crossing signs

### Encoder Feedback Missing
- Check `/VelocityEncL` and `/VelocityEncR` topics:
  ```bash
  ros2 topic list
  ros2 topic hz /VelocityEncL
  ```
- Verify hardware connections to encoders

---

## Topic Reference

| Topic | Type | Publisher | Subscriber(s) | Description |
|-------|------|-----------|---------------|-------------|
| `/video_source/compressed` | CompressedImage | Camera | image_viewer, ml_yolo, traffic_light | Compressed video stream |
| `/cmd_vel` | Twist | image_viewer, master | Robot driver | Movement commands (linear.x, angular.z) |
| `/line_lost` | Bool | image_viewer | master | True if line not detected |
| `/control_priority` | Bool | master | image_viewer | True if master has control |
| `/signal_detected` | Int32 | ml_yolo | master | Traffic sign ID (0-4) |
| `/light_state_int` | Int32 | traffic_light | master | Traffic light state (0-3) |
| `/light_state` | String | traffic_light | - | Traffic light color name |
| `/VelocityEncL` | Float32 | Encoder | image_viewer | Left wheel velocity (cm/s) |
| `/VelocityEncR` | Float32 | Encoder | image_viewer | Right wheel velocity (cm/s) |

---

## Project Structure

```
puzzlebot_ws/
├── best.pt                           # YOLOv8 trained model weights
├── README.md                         # This file
├── src/
│   └── puzzlebot/
│       ├── package.xml               # ROS 2 package manifest
│       ├── setup.py                  # Python package setup
│       ├── setup.cfg                 # Setup configuration
│       ├── resource/
│       │   └── puzzlebot             # Package marker
│       ├── launch/
│       │   └── traffic_light_launch.py    # Launch file for all nodes
│       ├── puzzlebot/
│       │   ├── __init__.py
│       │   ├── image_viewer_node.py       # Line following + dual PID
│       │   ├── ml_yolo.py                 # YOLO traffic sign detection
│       │   ├── traffic_light_node.py      # Traffic light detection (HSV)
│       │   └── master_node.py             # Hierarchical FSM controller
│       └── test/
│           ├── test_copyright.py
│           ├── test_flake8.py
│           └── test_pep257.py
├── build/                            # Colcon build artifacts
├── install/                          # Installed package files
└── log/                              # Build and runtime logs
```

---

## Control Flow Summary

1. **Normal Operation:**
   - `image_viewer_node` tracks line, publishes `/cmd_vel`
   - `ml_yolo` and `traffic_light_node` continuously monitor environment
   - `master_node` listens but doesn't interfere (`control_priority=False`)

2. **Sign Detected:**
   - YOLO publishes sign ID → `master_node` stores as pending action
   - Master waits for line to be lost (robot approaches intersection)

3. **Line Lost (Intersection):**
   - `image_viewer` publishes `line_lost=True`
   - `master_node` takes control (`control_priority=True`)
   - Executes appropriate maneuver (stop/cross/turn)
   - Returns control when line is reacquired

4. **Traffic Light Handling:**
   - Red/Yellow: Master transitions to STOPPED state
   - Green + Crossing sign: Execute crossing maneuver
   - No green light: Wait in STOPPED state

---

## Key Design Features

- **Hierarchical Control:** Two-tier control (line follower + behavioral FSM)
- **Debouncing:** Temporal filtering prevents noise-induced false triggers
- **Pending Actions:** Decouples sign detection from execution timing
- **Graceful Degradation:** Recovery behaviors for line loss
- **Multi-modal Sensing:** Combines computer vision (line), deep learning (signs), and color detection (lights)
- **Real-time Tuning:** PID and vision parameters adjustable without restart
- **Consistent Detection:** Rolling window filters for reliable sign recognition

---

## Dependencies

```xml
<!-- package.xml -->
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>launch</depend>
<depend>launch_ros</depend>
```

```bash
# Python packages
pip install opencv-python numpy torch ultralytics
```

---

## License

MIT

---

## Authors & Maintainers

Modify in [package.xml](src/semaforo/package.xml) and [setup.py](src/semaforo/setup.py):
- Maintainer: Jesus Martinez

---

## Future Improvements

- [ ] Add left turn support (currently only right turns)
- [ ] Implement dynamic speed adjustment based on curvature
- [ ] Add obstacle detection and avoidance
- [ ] Record and replay trajectories
- [ ] Web-based parameter tuning interface
- [ ] Multi-robot coordination
- [ ] Deep learning-based line following
- [ ] Adaptive PID auto-tuning
- [ ] RViz visualization of FSM states

---

## Debug Commands

```bash
# Monitor all topics
ros2 topic list

# Echo specific topic
ros2 topic echo /signal_detected
ros2 topic echo /light_state_int
ros2 topic echo /line_lost

# Check topic frequencies
ros2 topic hz /cmd_vel
ros2 topic hz /video_source/compressed

# View node graph
rqt_graph

# Monitor node info
ros2 node info /master_node
ros2 node info /image_viewer
```

---

**Happy Autonomous Driving! 🚗🤖**
