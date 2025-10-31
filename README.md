# Dual-LiDAR-LIO-SAM with IMU Enhancement (ROS2)

<p align="center">
  <img src="docs/demo.gif" alt="Dual-LiDAR-LIO-SAM Demo" width="80%">
</p>

---

## 📝 Overview

**Dual-LiDAR-LIO-SAM-with-IMU-Enhancement-ROS2** is an extended version of **LIO-SAM**, implemented in **ROS2**.  
The system provides **dual LiDAR support** through direct point cloud merging and introduces an **IMU enhancement module** that integrates a **Butterworth low-pass filter** and an **Extended Kalman Filter (EKF)** for more stable and accurate orientation estimation.  

This configuration slightly improves **pose stability and consistency** under mild vibration or uneven motion conditions, which often occur in outdoor or mobile robotic applications.

> ⚠️ **Note:**  
> This source does **not** perform calibration or time synchronization between the two LiDAR sensors.  
> Users must prepare and configure the extrinsic transformation between LiDAR A and LiDAR B beforehand.

---

## 🚀 Key Features

- 🔷 **Dual LiDAR merge** — directly merges point clouds from two LiDAR sensors to increase spatial coverage and density.  
- 🔶 **IMU noise suppression** — Butterworth low-pass filter effectively removes high-frequency vibration noise.  
- 🔹 **EKF-based orientation estimation** — enhances IMU attitude stability for odometry and mapping.  
- 💪 **Improved IMU stability** — the filtering and estimation pipeline slightly improves pose accuracy under vibration or rough motion.

---

**Processing Pipeline:**
1. LiDAR A + LiDAR B → Merge point clouds  
2. IMU → Butterworth Low-pass Filter → EKF Orientation Estimation  
3. Pose Estimation and Mapping via LIO-SAM framework  

---

## ⚙️ Installation

```bash
# 1. Clone this repository
cd ~/ros2_ws/src
git clone https://github.com/HoangHungIRL/Dual-LiDAR-LIO-SAM-with-IMU-Enhancement-ROS2.git

# 2. Build
cd ~/ros2_ws
colcon build

## ⚙️ Run algorithm

# 1. Source environment
source install/setup.bash

# 2. Run algorithm
ros2 launch lio_sam run.launch.py 
