# SVM Self‑Driving

A ROS 2‑based autonomous navigation stack for scaled vehicles that fuses LiDAR and depth‑camera data to generate SVM‑inspired virtual barriers and steer safely around obstacles.

---

## Overview

This project implements a real‑time obstacle detection and avoidance system on a 1:10 RC vehicle (MacAEV) using:

- **Sensor fusion** of a 2D SLAMTEC RPLIDAR A2M8 and an Intel RealSense D435 depth camera  
- **Ground‑plane filtering** (RANSAC) and **point‑cloud replacement** to merge camera and LiDAR ranges  
- **Convex optimization** to compute parallel virtual barriers that maximize clearance  
- **Finite‑state control** (normal, backup, turn) with a PD‑based steering controller for smooth navigation  

---

## System Architecture

![Architecture](/images/autonomous_architecture.png)

1. **RealSense Node** publishes `/camera/depth` and camera info  
2. **RPLIDAR Node** publishes `/scan`  
3. **VESC Driver & Odom Node** handle motor commands and odometry  
4. **Ackermann Mux Node** routes manual or autonomous `/drive` commands  
5. **Nav Node** subscribes to `/scan`, `/camera/depth`, and `/odom`, runs fusion & control, and publishes `/drive`  

---

## Sensor Fusion Pipeline

![Fusion Pipeline](/images/fusion_pipeline.png)

1. **Ground‑Plane Filtering**  
   - Apply RANSAC on the depth image to remove ground points  
2. **Frame Transformation**  
   - Transform remaining depth points into the LiDAR frame  
3. **Data Replacement**  
   - For each beam θᵢ, if camera range *r<sub>cam</sub>* < LiDAR range *r<sub>lidar</sub>*, replace *r<sub>lidar</sub>* with *r<sub>cam</sub>*  

This yields a fused 1D scan that captures obstacles both above and below the LiDAR plane.

---

## Barrier Computation & Control

![Barrier Diagram](/images/diagram.png)

- **Barrier Parameters**  
  - Compute heading θ<sub>head</sub> as weighted average of ranges  
  - Define left (α<sub>l</sub>, β<sub>l</sub>) and right (α<sub>r</sub>, β<sub>r</sub>) sectors  
- **Optimization**  
  Solve:
  ```math
   \begin{aligned}
   \min_{w,b}\ &\frac{1}{2}\|w\|^2 \\[6pt]
   \text{s.t.}\quad
   & w^\top p_i + b \ge 1, && \forall\,p_i \in \mathcal{P}_{\mathrm{left}},\\
   & w^\top p_j + b \le -1, && \forall\,p_j \in \mathcal{P}_{\mathrm{right}}.
   \end{aligned}
  ```
  This maximizes the margin (sum of left/right distances) between the two obstacle sets
- **Finite‑State Machine**  
  - **Normal:** drive forward, steer to maximize clearance (PD on *d<sub>lr</sub>*)  
  - **Backup:** reverse when blocked  
  - **Turn:** pivot in place if needed  

---

## Hardware Platform

![Vehicle Photo](/images/vehicle.png)

- **Chassis:** 1:10 RC Car (ARMA Granite 4×4)  
- **Compute:** NVIDIA Jetson Orin Nano AI computer  
- **Sensors:**  
  - SLAMTEC RPLIDAR A2M8 (2D LiDAR)  
  - Intel RealSense D435i (RGB‑D camera)  
  - BNO055 9‑DOF IMU  
  - TRAMPA VESC 6 MKV motor driver  
- **Power:** VENOM 3S LiPo battery  
- **Software:** Ubuntu 20.04, ROS 2 Foxy, C++17  

---

## Quick Start

1. **Install dependencies**
   ```bash
   sudo apt update
   sudo apt install \
     ros-foxy-ros-base \
     ros-foxy-ackermann-msgs \
     ros-foxy-rplidar-ros

2. To run the full system:

   ```bash
   ros2 launch auto_nav nav_launch.py
   
## Demo
> **Note**: The GIF may take a moment to load on GitHub.
![Demo](images/Self_Driving_RC_Car)
