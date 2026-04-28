# Mini-Project 13: Autonomous Global Path Planning (A* Algorithm)

## Overview
Navigation in autonomous robotics requires a robust global planner to find the optimal collision-free path from a start pose to a goal pose. This project implements the **A* (A-Star) search algorithm** entirely from scratch in Python, packaged as a standalone ROS 2 node.

## Algorithmic Architecture
The script operates on a 2D discrete grid (published as a `nav_msgs/OccupancyGrid`) where free space has a cost of $0$ and obstacles have a cost of $100$.

The algorithm evaluates nodes based on the standard A* evaluation function:

$$f(n) = g(n) + h(n)$$

Where:
* $n$: The current node being evaluated on the grid.
* $g(n)$: The exact cost of the path from the starting point to node $n$. (Orthogonal moves cost $1.0$, diagonal moves cost $\sqrt{2} \approx 1.414$).
* $h(n)$: The heuristic estimated cost from node $n$ to the goal. This implementation uses the **Manhattan Distance** heuristic:
  $$h(n) = |x_n - x_{goal}| + |y_n - y_{goal}|$$

## Implementation Details
* **Zero-Dependency Testing:** To ensure maximum reliability and ease of testing, the node procedurally generates its own simulated environment (a $20\times20$ map with L-shaped walls) upon initialization. 
* **Data Structures:** The Open List is managed using Python's `heapq` (priority queue) to guarantee $O(\log N)$ extraction of the lowest-cost node, making the algorithm highly efficient even on larger grids.
* **Visualization:** The computed route is published as a `nav_msgs/Path`. By running `rviz2` and adding the `/costmap` and `/planned_path` topics, the optimal obstacle-avoidance trajectory can be visualized in real-time.
