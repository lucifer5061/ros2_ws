# ROS 2 Sensor Emulator and Cue Fusion System

This repository contains a custom ROS 2-based sensor emulator and cue fusion system developed to simulate and test multi-sensor perception for an autonomous drone or robotic surveillance system. The system is designed in two major milestones, offering modular sensor emulation, configurable motion profiles, and real-time latency measurement with visual feedback.

---

## 🔧 Features

### ✅ Simulated Sensor Streams
| Topic | Description |
|-------|-------------|
| `/gps/fix` | Simulated GNSS coordinates |
| `/imu/data` | Simulated IMU data including orientation and linear acceleration |
| `/mock/rf_cue` | RF-based target cue with RSSI (signal strength) |
| `/mock/visual_cue` | Visual detection cue (e.g., from camera/ML model) |
| `/synthetic_target_cue` | Fused detection from visual + RF cues |

---

### ✅ Motion Profiles (Configurable via YAML)
Supports various motion types for simulating target behavior:

- `linear` – Constant velocity, straight-line motion  
- `zigzag` – Lateral oscillation, useful for evasive targets  
- `circle` – Circular or patrol-like movement  
- `hover` – Mostly static target with slight drift  

Configure motion type and parameters in:  
`config/motion_profile.yaml`

---

## 🧠 Architecture Overview

### Nodes

#### `sensor_emulator_node.py` 
Publishes `/gps/fix`, `/imu/data`, and `/track/fake_target`.

#### `test_input_publisher.py`
- Publishes `/mock/rf_cue` and `/mock/visual_cue`.
- Injects **valid and invalid cues** at a fixed rate (10 Hz).
- Every few seconds, publishes matched RF and visual detections for fusion testing.

#### `cue_fusion_node.py`
- Subscribes to `/mock/visual_cue` and `/mock/rf_cue`
- Fuses cues based on:
  - Spatial distance ≤ 2.0 meters
  - Time difference ≤ 0.1 seconds
  - `class_id == 1` (for visual)
  - `RSSI ≥ 0.5` (for RF)
- Publishes fused output to `/synthetic_target_cue`
- Logs successful fusions to `logs/detections.csv`

---

## 🚀 Launch Instructions

Make sure your workspace is built and sourced:

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_emulator --merge-install
source install/setup.bash
ros2 launch sensor_emulator sensor_emulator_launch.py
```
---


## 📌 Notes
  - The older imu_profile.yaml has been deprecated and replaced by a unified motion_profile.yaml.

  - Sensor timestamps and orientations are simulated with fixed covariance matrices for simplicity.

  - Cue publishing is deterministic and reproducible.
    
  - You can easily switch motion profiles via motion_profile.yaml.

  - cue_fusion_node.py only fuses the first valid pair per cycle to avoid spamming.
 
  - All messages are published as std_msgs/String with JSON payloads for flexibility.


---

## 🎯 Milestone 1 – Deliverables
  - ✅ Simulated RF and visual target cues with control over match frequency
  - ✅ Fusion node that detects valid matches based on confidence, RSSI, and time-space proximity
  - ✅ Fusion results published on /synthetic_target_cue
  - ✅ CSV-based logging of fused target detections
  - ✅ Configurable motion logic in YAML

## 🎯 Milestone 2 – Deliverables
  - ✅ Real-time latency measurement between input cues and output fused detections
  - ✅ Automatic logging of fusion output and latency to `detections.csv`
  - ✅ Visualization of fused target positions (X-Y plane) via `plot_fused_detections.py`
  - ✅ Fully integrated launch system to run the complete simulation
  

## 👨‍💻 Author
  - Syed Ahmed Zulfiqar
  - Final Year Electronics Engineering Student – NED University
  - syedahmed.zulfiqar@gmail.com
  - GitHub: @lucifer5061
