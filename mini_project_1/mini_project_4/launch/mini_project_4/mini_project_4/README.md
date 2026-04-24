# Mini-Project 4: Multi-Robot & Fleet Management

## Overview
This project addresses the challenges of managing multiple robotic units within a shared environment. It focuses on **namespace isolation** and **centralized coordination**.

## Key Concepts
* **Namespace Isolation:** Preventing topic collisions by wrapping each robot's nodes in a specific namespace (`/robot1`, `/robot2`).
* **Fleet Coordination:** A centralized `FleetCommander` node that monitors robot states and assigns navigation tasks.
* **Launch Architecture:** Advanced use of `GroupAction` and `PushRosNamespace` for scalable deployment.

## System Architecture
The system uses a **Fleet Commander** to communicate with multiple instances of `nav2` and `robot_state_publisher`.

```bash
# Example of checking fleet status
ros2 topic echo /fleet/status
