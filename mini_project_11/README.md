# Mini-Project 11: AI Target Tracking & Visual Servoing (Terminator Mode)

## Overview
This project represents the convergence of **Computer Vision**, **Control Theory**, and **ROS 2 Robotics**. It implements a complete Visual Servoing loop: the robot uses a camera feed to detect human faces (using OpenCV Haar Cascades), calculates the spatial error, and dynamically adjusts its velocity to intercept the target.

## Control Systems Architecture
The robot uses a **Closed-Loop Proportional Controller** based on visual feedback.

### 1. Angular Tracking (Pan Control)
To keep the target perfectly centered in its Field of View (FOV), the node calculates the pixel error between the image center ($X_{center}$) and the target bounding box center ($X_{target}$). The angular velocity $\omega_z$ is generated using:

$$\omega_z = K_{p, \text{angular}} \times (X_{center} - X_{target})$$

### 2. Linear Interception (Depth Control)
To approach the target, the node measures the pixel width $W$ of the bounding box. A smaller bounding box means the target is far away. The linear velocity $V_x$ is generated using:

$$V_x = K_{p, \text{linear}} \times (W_{target} - W)$$

## Hardware/Simulation Requirements
* A robot chassis equipped with a camera (publishing to `/camera/image_raw`).
* The node handles `cv_bridge` conversions to seamlessly bridge the ROS 2 environment with the native `OpenCV` C++ backend.
* **Dependencies:** `rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `opencv-python`.

## Mission Result
When launched, the robot spins on its Z-axis ("Searching" mode). Once a target enters the camera frame, the robot locks onto the coordinates, centers the target, and drives forward continuously until intercept distance is reached.
