# Mini-Project 19: Extended Kalman Filter (EKF) Sensor Fusion

## Overview
Accurate state estimation is a fundamental requirement for autonomous robotics. Raw sensor data is inherently flawed: wheel odometry accumulates drift over time (dead reckoning error), and GPS signals suffer from high variance and multipath noise. 

This project implements an **Extended Kalman Filter (EKF)** from scratch to fuse noisy kinematic control inputs with noisy observation data, achieving an optimal, probabilistically sound estimation of the robot's state in a 2D plane.

## Mathematical Architecture
Because the kinematic bicycle model is non-linear (involving trigonometric functions for orientation), a standard Kalman Filter cannot be used. The EKF linearizes the system around the current mean estimate using first-order Taylor series expansion via **Jacobian Matrices**.

### 1. Predict Step (Motion Model Update)
The state $x$ and the error covariance $P$ are predicted based on the previous state and control input $u$:

$$x_{k|k-1} = f(x_{k-1|k-1}, u_k)$$
$$P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q$$

Where $F_k$ is the Jacobian matrix of partial derivatives of the state transition function $f$, and $Q$ is the process noise covariance.

### 2. Update Step (Sensor Correction)
When a new GPS measurement $z_k$ arrives, the filter calculates the Kalman Gain $K_k$, which acts as an optimal weighting factor between the prediction and the measurement:

$$y_k = z_k - h(x_{k|k-1}) \quad \text{(Measurement Residual)}$$
$$S_k = H_k P_{k|k-1} H_k^T + R \quad \text{(Residual Covariance)}$$
$$K_k = P_{k|k-1} H_k^T S_k^{-1} \quad \text{(Kalman Gain)}$$

The state and covariance are then refined:
$$x_{k|k} = x_{k|k-1} + K_k y_k$$
$$P_{k|k} = (I - K_k H_k) P_{k|k-1}$$

## Implementation Details
This ROS 2 node is fully self-contained. Upon execution, it dynamically simulates a robot traversing a complex trajectory, injects artificial Gaussian noise into both the internal odometry ($Q$ matrix) and the external sensor readings ($R$ matrix). It computes the massive matrix multiplications at 10 Hz. 

At the end of the simulation, it outputs a quantitative analysis to the terminal, proving mathematically that the EKF reduces the tracking error by an order of magnitude compared to dead reckoning.
