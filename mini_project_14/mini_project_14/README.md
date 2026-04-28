# Mini-Project 14: LIDAR-Based Occupancy Grid Mapping

## Overview
Mapping is the process by which a robot builds a spatial representation of its environment. This project implements a **Stochastic Occupancy Grid Mapper** using raw range-finder data. It utilizes the **Bresenham Line Algorithm** for fast raycasting to update the probability of occupancy for each cell.



## Technical Features
### 1. Log-Odds Belief Update
Instead of simple binary values, this mapper uses a probabilistic approach. Each cell $(x,y)$ stores a "belief" value. When a laser ray passes through a cell, its occupancy probability decreases. When a ray hits an obstacle, the probability increases.

### 2. Raycasting Architecture
The implementation features a high-performance Raycasting engine. For every laser return, the node calculates the exact sequence of grid cells traversed from the robot's center to the impact point:
* **Free Space:** Detected via inverse sensor model along the ray.
* **Occupied Space:** Detected at the ray's terminal point.

## Visualization
The resulting map is published on the `/map` topic as a `nav_msgs/OccupancyGrid`. It is fully compatible with **RViz2** and can be used as a static map for the ROS 2 Navigation Stack (Nav2).
