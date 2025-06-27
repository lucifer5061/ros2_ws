# 📡 Sensor Emulator – ROS 2 Package for Autonomous Anti-Drone System

The `sensor_emulator` package provides simulated GPS, IMU, and target detection data streams for testing and developing autonomous anti-drone systems in a multi-node ROS 2 environment. It supports dynamic motion patterns and allows developers to test perception, tracking, and control logic without needing physical sensors.

---

## 🔧 Features

- ✅ **Simulated Sensor Streams**
  - `/gps/fix` – GNSS coordinates
  - `/imu/data` – IMU data (orientation, linear acceleration)
  - `/track/fake_target` – Target detections (e.g., from radar or vision)

- ✅ **Motion Types (Configurable)**
  - `linear` – Straight line movement
  - `zigzag` – Rapid lateral oscillation
  - `circle` – Circular or patrol motion
  - `hover` – Static with small drift (e.g., quadrotor hover)

- ✅ **Flexible Launch System**
  - Launch via ROS 2 launch file with `motion:=` argument
  - Easily extendable for custom profiles

---

## 🚀 Launch Instructions

Make sure your workspace is built and sourced:

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_emulator
source install/setup.bash
```
---

- 🧠 Architecture Overview
  - sensor_emulator_node.py – Main ROS 2 node that publishes sensor messages

  - motion_profiles.py – Implements different motion models

  - motion_profile.yaml – Central configuration for motion parameters

  - Launch files and test scaffolding included

---

- 📌 Notes
  - The older imu_profile.yaml has been deprecated and replaced by a unified motion_profile.yaml.

  - Sensor timestamps and orientations are simulated with fixed covariance matrices for simplicity.

  - This package is under active development – Week 3 will add fused detections and target motion

---

- 👨‍💻 Author
  - Syed Ahmed Zulfiqar
  - Final Year Electronics Engineering Student – NED University
  - GitHub: @lucifer5061
