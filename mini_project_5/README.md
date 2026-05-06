# Mini-Project 5: Zenoh Edge-to-Cloud Communication

## Overview
Moving beyond local DDS, this project implements **Zenoh** to allow ROS 2 communication over WAN and Cloud. It features a Bridge configuration that filters high-bandwidth topics to optimize data transmission between the robot (Edge) and the server (Cloud).
# 🗺️ Projet 05 : Gestion des Cartes et Serveur de Navigation

## 📋 Description
Ce package est responsable du chargement et de la diffusion de la carte de l'environnement (Occupancy Grid). Il permet au robot de se situer dans un plan connu et définit les obstacles statiques via le serveur de carte de Nav2.

## 🛠️ Prérequis
* ROS 2 (Humble/Iron)
* Plugin `nav2_map_server`

## ⚙️ Installation & Compilation
1. Clonez ce dossier dans votre workspace.
2. Compilez :
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select mini_project_05
