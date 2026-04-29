# Mini-Project 15: Monte Carlo Localization (Particle Filter)

## Overview
Global localization is one of the most complex problems in autonomous robotics. This project implements a **Monte Carlo Localization (MCL)** algorithm, a recursive Bayes filter that estimates the posterior distribution of a robot's pose using a set of weighted particles. 

This node is completely self-contained, simulating the kinematic updates and sensor models entirely via matrix operations in `numpy`, and publishing the resulting particle cloud as a `geometry_msgs/PoseArray` for RViz2 visualization.

## Mathematical Architecture

The algorithm operates in a continuous three-step Markov chain:

### 1. Motion Prediction (Kinematic Model)
When the robot moves, every particle $x_t^{[m]}$ is updated according to the control command $u_t$ and a Gaussian noise distribution simulating wheel slippage:

$$x_t^{[m]} \sim p(x_t | u_t, x_{t-1}^{[m]})$$

### 2. Observation Update (Sensor Model)
Upon receiving simulated range measurements to a known landmark $z_t$, the algorithm calculates the importance weight $w_t^{[m]}$ of each particle. The weight represents the likelihood of receiving that exact measurement if the robot were actually located at the particle's coordinates:

$$w_t^{[m]} \propto p(z_t | x_t^{[m]}, m)$$

### 3. Systematic Resampling (Survival of the Fittest)
To prevent particle deprivation, the algorithm draws $N$ new particles from the current set, with the probability of drawing a particle being directly proportional to its weight. Low-weight particles (impossible locations) are destroyed, while high-weight particles (highly probable locations) are duplicated.

## Technical Merits
* **Vectorization:** Instead of slow `for-loops` to calculate physics for 500 particles, the prediction step leverages full `numpy` array vectorization, allowing massive parallel calculations crucial for high-frequency control loops.
* **Kidnapped Robot Recovery:** The code includes safety fallbacks to automatically re-distribute particles uniformly across the map if the sensor weights drop to zero (simulating a scenario where the robot is physically picked up and moved).
