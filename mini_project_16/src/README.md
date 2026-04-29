# Mini-Project 16: Artificial Potential Fields (APF) Reactive Navigation

## Overview
While algorithms like A* or Dijkstra are excellent for global path planning, they struggle in highly dynamic environments. This project implements an **Artificial Potential Fields (APF)** controller for real-time, reactive obstacle avoidance. 

The robot is treated as a particle moving under the influence of an artificial potential field $U(q)$, which is the superposition of an attractive potential towards the goal and a repulsive potential away from obstacles.

## Mathematical Foundation (Khatib's Model)

### 1. Attractive Potential Field
The robot is constantly pulled towards the target coordinates $q_{goal}$ by a parabolic attractive well. The attractive force $F_{att}$ is the negative gradient of the attractive potential:

$$F_{att} = -\nabla U_{att}(q) = -k_{att}(q - q_{goal})$$
*(Where $k_{att}$ is the attractive scaling gain).*

### 2. Repulsive Potential Field
To prevent collisions, obstacles generate a repulsive force $F_{rep}$ that pushes the robot away. This force is inversely proportional to the square of the distance to the obstacle $\rho(q)$, but only active within a distance of influence $\rho_0$:

$$F_{rep} = \begin{cases} k_{rep}\left(\frac{1}{\rho(q)} - \frac{1}{\rho_0}\right)\frac{1}{\rho(q)^2}\nabla\rho(q) & \text{if } \rho(q) \leq \rho_0 \\ 0 & \text{if } \rho(q) > \rho_0 \end{cases}$$

### 3. Resultant Force and Kinematics
The total force vector dictates the desired heading of the robot:
$$F_{total} = F_{att} + \sum F_{rep}$$

A custom proportional kinematic controller translates this 2D vector $(F_x, F_y)$ into continuous `cmd_vel` (`Twist`) commands, smoothly adjusting both the linear velocity $v$ and angular velocity $\omega$ to navigate complex obstacle clusters without stopping.

## Implementation Notes
To ensure flawless zero-dependency execution, this ROS 2 node encapsulates a built-in physics simulator. It independently updates its own Cartesian coordinates using Euler integration, allowing it to calculate forces, avoid internal arrays of obstacles, and successfully complete its mission merely by running the script.
