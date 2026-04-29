# Mini-Project 18: Iterative Inverse Kinematics (Jacobian Pseudo-Inverse)

## Overview
Calculating the precise joint angles required for a robotic arm to reach a specific $(x, y, z)$ spatial coordinate is known as the Inverse Kinematics (IK) problem. While analytical solutions exist for simple geometries, complex architectures (like a 4-DOF RRRR manipulator) require iterative numerical solvers. 

This standalone ROS 2 node implements a numerical IK solver utilizing the **Moore-Penrose Pseudo-Inverse of the Jacobian matrix**.

## Mathematical Formulation

### 1. Forward Kinematics (FK)
The position of the end-effector $X$ is a non-linear function of the joint angles $q$:
$$X = f(q)$$

### 2. The Jacobian Matrix
The Jacobian $J$ represents the differential relationship between joint velocities and end-effector velocities. It is calculated numerically using finite differences:
$$J(q) = \frac{\partial f(q)}{\partial q}$$

### 3. Iterative Update Law
To minimize the spatial error $e = X_{target} - X_{current}$, the algorithm calculates the required change in joint angles $\Delta q$. Because the Jacobian is rarely square (e.g., a 4-DOF arm in 3D space yields a $3 \times 4$ matrix), we cannot invert it directly. Instead, we use the Moore-Penrose pseudo-inverse $J^{\dagger}$:
$$\Delta q = J^{\dagger} e$$

The joints are then updated iteratively scaled by a learning rate $\alpha$:
$$q_{k+1} = q_k + \alpha \Delta q$$

## Implementation Details
* **Zero-Dependency Execution:** The node contains its own internal simulation loop for an RRRR manipulator ($L_1=0.4m, L_2=0.3m, L_3=0.3m, L_4=0.2m$). It requires no physical hardware or external simulation to execute.
* **Singularity Robustness:** By using `numpy.linalg.pinv`, the solver intrinsically handles kinematic singularities (positions where the arm loses a degree of freedom) without crashing.
* **ROS Integration:** The computed angles are continuously broadcasted to the `/joint_states` topic, making it fully ready to drive a URDF model in `robot_state_publisher` and RViz2.
