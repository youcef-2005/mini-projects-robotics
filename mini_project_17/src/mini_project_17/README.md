# Mini-Project 17: Model Predictive Control (MPC) Trajectory Tracking

## Overview
Model Predictive Control (MPC) is an advanced method of process control that is used to control a system while satisfying a set of constraints. Unlike traditional PID controllers, MPC uses a **multivariable control strategy** that predicts future events and takes control actions accordingly.



## How it Works: The Receding Horizon
At each time step $k$, the controller solves a finite-horizon mathematical optimization problem:
1.  **State Prediction:** Using a kinematic model, it simulates $N$ steps into the future.
2.  **Cost Minimization:** It calculates a cost function $J$ that penalizes the distance to the reference path and the control effort:
    $$J = \sum_{i=1}^{N} Q \|x_{k+i} - x_{ref,k+i}\|^2 + R \|u_{k+i}\|^2$$
3.  **Action:** Only the first optimal control action is applied, and the entire process is repeated at the next time step.

## Technical Merits
* **Native Implementation:** This node features a custom-built iterative solver that requires no external heavy optimization libraries (like Ipopt or CasADi), making it extremely robust and easy to deploy on embedded systems.
* **Predictive Capability:** By considering future reference points, the robot "anticipates" curves, slowing down before sharp turns and accelerating on straight lines, resulting in much smoother motion than reactive methods.
