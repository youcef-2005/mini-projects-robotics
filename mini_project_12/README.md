# Mini-Project 12: Extended Kalman Filter (EKF) Sensor Fusion

## Overview
In real-world robotics, sensors like wheel encoders (Odometry) suffer from cumulative drift, while Inertial Measurement Units (IMUs) suffer from high-frequency noise. This node solves this by implementing a custom **Extended Kalman Filter (EKF)** from scratch using `numpy`. It probabilistically fuses `/odom_noisy` and `/imu/data` to output a highly accurate `/odom_fused` state.

## Mathematical Formulation
The EKF linearizes the non-linear kinematic model of a differential drive robot using Jacobian matrices. The internal state vector is defined as $X = [x, y, \theta, v, \omega]^T$.

### 1. Predict Step (Kinematic Model)
The prior state estimate is calculated based on the previous state and the time step $dt$:

$$X_{k|k-1} = F(X_{k-1|k-1})$$

The covariance matrix $P$ is updated using the Jacobian $F$ and the process noise matrix $Q$:

$$P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q$$

### 2. Update Step (Measurement Model)
When new sensor data arrives, the Kalman Gain $K$ is computed to weigh the trust between the mathematical prediction and the actual noisy measurements ($Z$):

$$K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}$$

The final posterior state and covariance are then updated:

$$X_{k|k} = X_{k|k-1} + K_k (Z_k - H X_{k|k-1})$$
$$P_{k|k} = (I - K_k H) P_{k|k-1}$$

## Implementation Details
Unlike standard ROS packages (`robot_localization`), this node writes the matrix multiplications manually. This provides a deep understanding of control theory and probabilistic state estimation, which is crucial for advanced industrial automation and autonomous navigation systems.
