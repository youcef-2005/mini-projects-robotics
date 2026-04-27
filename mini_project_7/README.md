# Mini-Project 7: Production-Grade Lifecycle Management

## Overview
Standard ROS 2 nodes start executing immediately upon launch, which can lead to race conditions, undefined behaviors, or hardware crashes in complex industrial systems. This project implements a **Lifecycle Node (Managed Node)** to provide deterministic startup, shutdown, and error-handling routines.

## State Machine Architecture
This node strictly follows the ROS 2 lifecycle state machine, ensuring safe hardware integration:
1. **Unconfigured:** The node is launched but consumes minimal memory. No hardware drivers are initialized.
2. **Inactive (Configured):** Memory is allocated, parameters are loaded, and hardware checks are performed, but no network traffic is generated.
3. **Active:** Full operation. Topics are published, and lasers/actuators are enabled.
4. **Finalized:** Safe shutdown state to prevent hardware damage.

## How to test state transitions (CLI)

Unlike standard nodes, this node is controlled via the ROS 2 Lifecycle CLI.

**1. Start the Node:**
```bash
ros2 run my_package industrial_sensor_lifecycle
