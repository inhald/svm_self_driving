# ROS2 Autonomous Navigation

## Overview

This project implements a ROS 2-based autonomous navigation system that fuses point cloud data from a **SLAMTEC RPLIDAR A2M8** and an **Intel RealSense D435** depth camera. The fused data is used to detect obstacles in real time, generate **SVM-inspired virtual barriers**, and steer the robot along safe, collision-free paths.

## Sensor Fusion Pipeline

The system merges spatial data from both the RPLIDAR and RealSense camera into a unified point cloud. It applies preprocessing (filtering, downsampling, frame transforms), then extracts obstacle boundaries using a linear approximation of the maximum-margin separator (SVM-inspired). The output is a virtual barrier that defines the safe traversal corridor for the robot.

**Key components of the pipeline include:**

- **RealSense Node**: Streams aligned RGB and depth data.  
- **RPLIDAR Node**: Publishes 2D laser scans.

## Launch Instructions

To run the full system:

```bash
ros2 launch auto_nav nav_launch.py

## Demo
> **Note**: The GIF may take a moment to load on GitHub.
![Demo](https://github.com/inhald/svm_self_driving/blob/main/optimized-compression.gif)
