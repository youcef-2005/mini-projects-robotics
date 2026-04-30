# Mini-Project 21: Digital Twin & Discrete-Time Control (Z-Transform)

## Overview
While ROS 2 operates as a high-level software framework, low-level robotic actuators (like DC motors) are controlled by embedded microcontrollers. These embedded systems cannot compute continuous calculus; they must sample data and process algorithms in discrete time steps. 

This project implements a **Digital Twin Architecture**. It internally simulates the continuous-time physics of a DC Motor using Euler integration, injects realistic Gaussian sensor noise, and controls the system using a strictly discrete-time Digital Signal Processing (DSP) pipeline and PID controller.

## Mathematical Formulation

### 1. The Continuous Physical Plant (Laplace Domain)
The simulated DC motor is modeled as a first-order system with a static gain $K$ and a time constant $\tau$. Its continuous transfer function in the Laplace domain is:

$$G(s) = \frac{V(s)}{U(s)} = \frac{K}{\tau s + 1}$$

### 2. The Digital Low-Pass Filter (Z-Domain)
Raw sensor measurements are corrupted by high-frequency Gaussian noise. Before entering the controller, the signal passes through a first-order Infinite Impulse Response (IIR) digital low-pass filter. The transfer function in the Z-domain is:

$$H(z) = \frac{\alpha}{1 - (1 - \alpha)z^{-1}}$$

Converted into a recursive difference equation executable by an embedded CPU:
$$y[k] = \alpha x[k] + (1 - \alpha) y[k-1]$$

### 3. The Discrete PID Controller
The classic continuous PID equation is discretized to run at a fixed sampling interval $\Delta t = 0.05 \text{s}$. 
* **Integration** is approximated using the forward Euler method.
* **Differentiation** is approximated using first-order backward finite differences.

$$u[k] = K_p e[k] + K_i \sum_{i=0}^{k} e[i]\Delta t + K_d \frac{e[k] - e[k-1]}{\Delta t}$$

## Execution & Resilience
This node is engineered as a zero-dependency, self-terminating validation script. Upon launch, it executes the control loop, printing a real-time dashboard to the console. Once the transient step-response stabilizes (at $t = 5.0\text{s}$), the system automatically shuts down the virtual physical plant. It is $100\%$ reliable out of the box.
