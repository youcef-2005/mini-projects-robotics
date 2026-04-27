# Mini-Project 9: 4-DOF RRRR Manipulator Forward Kinematics (Denavit-Hartenberg)

## Overview
This advanced node bridges the gap between mechanical design and software control. It computes the real-time 3D spatial coordinates (Forward Kinematics) of a custom 4-Degree-of-Freedom (RRRR) robotic manipulator arm using the standard Denavit-Hartenberg (DH) convention. 

This node is designed to ingest `sensor_msgs/JointState` data (from simulated hardware or real encoders) and output the exact spatial location via `geometry_msgs/PoseStamped`.

## Mathematical Foundation
The calculation relies on computing the homogeneous transformation matrices for each link. The general DH transformation matrix $A_i$ from frame $i-1$ to frame $i$ is defined as:

$$A_i = \begin{bmatrix} \cos(\theta_i) & -\sin(\theta_i)\cos(\alpha_i) & \sin(\theta_i)\sin(\alpha_i) & a_i\cos(\theta_i) \\ \sin(\theta_i) & \cos(\theta_i)\cos(\alpha_i) & -\cos(\theta_i)\sin(\alpha_i) & a_i\sin(\theta_i) \\ 0 & \sin(\alpha_i) & \cos(\alpha_i) & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Where the parameters represent:
* $\theta_i$: Joint angle (rotation around Z)
* $d_i$: Link offset (translation along Z)
* $a_i$: Link length (translation along X)
* $\alpha_i$: Link twist (rotation around X)

The final end-effector pose relative to the base frame is obtained by multiplying the four matrices: $T_{0}^{4} = A_1 A_2 A_3 A_4$.

## Industrial Application
By structuring the code this way, the ROS 2 node becomes agnostic to the physical robot. Whether the mechanical model is built in SOLIDWORKS or Gazebo, as long as the physical lengths ($L_1 = 0.5m$, $L_2 = 0.4m$, $L_3 = 0.2m$, $L_4 = 0.1m$) match the parameters in `forward_kinematics_rrrr.py`, the spatial tracking will be mathematically perfect.
