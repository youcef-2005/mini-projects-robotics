# Mini-Project 5: Zenoh Edge-to-Cloud Communication (ROS 2 Jazzy)

## Overview
This project demonstrates how to bypass traditional DDS limitations over WAN/Internet using **Zenoh**. It sets up a Bridge to allow ROS 2 nodes to communicate across different local networks seamlessly.

## Architecture
* **Edge (Robot):** Runs `zenoh_bridge_ros2dds` in client mode. Filters high-bandwidth topics (like LiDAR and Cameras) to save data.
* **Cloud (Server):** Runs `zenoh_bridge_ros2dds` in router mode, exposing a REST API for web-based monitoring.

## Deployment Commands

### 1. Direct RMW Implementation
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp zenoh_router
