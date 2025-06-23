# ROS 2 Anti-Drone System

This project is a ROS 2-based sensor emulator for simulating IMU data as part of a larger anti-drone system.

---

## 📦 Package: `sensor_emulator`

This node publishes simulated IMU messages to `/imu/data`.

### 🔁 Behavior:
- Publishes data at 10 Hz
- Simulates yaw rotation (Z-axis)
- Constant angular velocity and gravity vector

---

## 🚀 Setup Instructions

### 1. Clone and build

```bash
cd ~/ros2_ws/src
git clone https://github.com/lucifer5061/ros2_ws.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

