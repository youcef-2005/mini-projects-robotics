# Mini-Project 10: 10km Autonomous Flight Simulation (UAV/Helicopter)

## Overview
This project simulates a long-range autonomous flight mission for a vertical take-off and landing (VTOL) aircraft. The goal is to perform a **10-kilometer cross-country flight** at a stabilized altitude of 50 meters.

## Flight Profile
The mission is divided into three automated phases:
1.  **Take-off Phase:** Vertical thrust is applied until the target altitude of 50m is reached.
2.  **Cruise Phase:** The autopilot maintains a forward velocity of 25 m/s (~90 km/h) while constantly adjusting the vertical stabilizer to compensate for gravity/drift.
3.  **Landing Phase:** Upon reaching the 10,000m mark, the forward velocity is cut, and a controlled descent is initiated.

## Control Logic
The autopilot uses a simple yet effective proportional control for altitude maintenance:

$$V_z = K_p \cdot (Z_{target} - Z_{current})$$

This ensures that the helicopter stays within its flight corridor throughout the 10km transit.

## Technical Stack
* **Simulation:** Gazebo with a generic UAV/Helicopter plugin.
* **Navigation:** Odometry-based spatial tracking.
* **Communications:** ROS 2 Humble/Jazzy.
# 🚀 Projet 10 : Automatisation avec Fichiers Launch

## 📋 Description
Ce package introduit l'utilisation des fichiers `launch.py`. Plutôt que de lancer chaque nœud manuellement dans un terminal différent, nous utilisons un script de lancement qui coordonne le démarrage de plusieurs nœuds simultanément, avec des configurations spécifiques.

## 🛠️ Prérequis
* ROS 2 (Humble/Iron)
* Packages `launch` et `launch_ros`

## ⚙️ Installation & Compilation
1. Clonez ce dossier dans votre workspace.
2. Compilez :
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select mini_project_10
