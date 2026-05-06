# Mini-Project 4: Multi-Robot Launch & Namespace Isolation

## Overview
This project demonstrates how to launch multiple robot instances (Robot1 & Robot2) within a single launch file using **Namespaces**. This prevents topic collisions (like having two `/cmd_vel`) and allows a fleet manager to control each robot independently.
# 🐝 Projet 04 : Gestion Multi-Robot et Namespaces

## 📋 Description
Ce package prépare l'architecture pour la robotique en essaim (Swarm Robotics). Il démontre comment lancer plusieurs instances d'un même nœud de contrôle en utilisant les Namespaces de ROS 2 pour éviter les collisions de noms de topics et de services.

## 🛠️ Prérequis
* ROS 2 (Humble/Iron)
* Notions sur les graphes ROS

## ⚙️ Installation & Compilation
1. Clonez ce dossier dans votre workspace.
2. Compilez :
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select mini_project_04
