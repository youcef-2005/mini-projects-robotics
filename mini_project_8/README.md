# Mini-Project 8: Closed-Loop Distance Controller

## Overview
This project implements a mobile robot navigation node. Instead of relying on open-loop time-based movement (which is highly susceptible to battery levels, friction, and slippage), this node uses a **closed-loop control strategy** based on real-time Odometry feedback.

## Objectives
* Command a mobile robot to move strictly forward for exactly 50.0 meters.
* Publish velocity commands via `geometry_msgs/Twist` to the `/cmd_vel` topic.
* Subscribe to `nav_msgs/Odometry` on the `/odom` topic to track coordinates.
* Implement automatic braking once the target is reached.

## Mathematical Model
The displacement is calculated continuously using the Euclidean distance formula between the initial pose $(x_0, y_0)$ and the current pose $(x_c, y_c)$:

$$d = \sqrt{(x_c - x_0)^2 + (y_c - y_0)^2}$$

The node maintains a constant linear velocity $v_x = 0.8 \text{ m/s}$ as long as $d < 50.0$, and sends a zero-velocity command ($v_x = 0.0$) the moment the condition is met.

## How to Run
This node can be tested directly in a Gazebo simulation (e.g., TurtleBot3, Clearpath Husky, or any custom URDF model publishing odometry).

```bash
ros2 run my_package distance_controller
# 📉 Projet 08 : Contrôleur PID et Asservissement

## 📋 Description
Ce package implémente un algorithme de contrôle PID. Il permet de corriger l'erreur entre la position actuelle d'un moteur et sa position cible en ajustant les paramètres Proportionnel, Intégral et Dérivé pour obtenir une réponse rapide et stable.

## 🛠️ Prérequis
* ROS 2 (Humble/Iron)
* Notions de base en automatique (Transfer functions)

## ⚙️ Installation & Compilation
1. Clonez ce dossier dans votre workspace.
2. Compilez :
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select mini_project_08
