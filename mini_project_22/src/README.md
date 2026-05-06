# Mini-Project 22: Decentralized Swarm Robotics (Multi-Agent Systems)

## Overview
Traditional robotics relies on centralized control architectures, which create single points of failure. This project implements a **Multi-Agent System (MAS)** framework utilizing the Reynolds Boids algorithm. It simulates an emergent, decentralized swarm of 50 autonomous robots operating concurrently. 

There is no central orchestrator; each agent computes its own kinematic state based strictly on local sensor data (distance to immediate neighbors), proving that complex global behavior can emerge from simple local rules.

## The Three Rules of Flocking Dynamics
The acceleration vector for any given robot $i$ is calculated via vector addition of three distinct social forces. Let $N_i$ be the set of neighboring robots within the visual range.

### 1. Separation (Collision Avoidance)
Robots are repulsed by neighbors that breach a critical distance threshold $d_{protected}$, ensuring physical safety.
$$F_{sep, i} = k_{sep} \sum_{j \in N_{close}} (X_i - X_j)$$

### 2. Alignment (Velocity Matching)
Robots attempt to match their velocity vectors with the local flock's average velocity, creating synchronized traffic flow.
$$F_{align, i} = k_{align} \left( \frac{1}{|N_i|} \sum_{j \in N_i} V_j - V_i \right)$$

### 3. Cohesion (Flock Centering)
Robots steer towards the geometric center of mass of their local neighborhood, ensuring the swarm does not disperse.
$$F_{coh, i} = k_{coh} \left( \frac{1}{|N_i|} \sum_{j \in N_i} X_j - X_i \right)$$

## Technical Implementation
To compute $O(N^2)$ distance checks per cycle at 20 Hz without CPU bottlenecking, the node abandons standard Python `for-loops` in favor of multidimensional `numpy` matrix broadcasting. 

The entire $50$-agent swarm state is continuously published to a single `geometry_msgs/PoseArray` topic, allowing for real-time, low-latency visualization of the entire flock within RViz2.
## 🚀 Commandes de Lancement (Instructions d'exécution)

Pour visualiser la simulation de l'essaim, ouvrez deux terminaux distincts. N'oubliez pas de sourcer votre environnement (`source install/setup.bash`) dans chaque terminal.

### [Terminal 1] - Lancement de la simulation Swarm
Ce nœud calcule les comportements de séparation, d'alignement et de cohésion pour les 50 robots, et publie leurs positions à 20 Hz.
`ros2 run mini_project_22 swarm_node`

### [Terminal 2] - Lancement de la visualisation RViz2
Affiche les robots en temps réel.
`rviz2`
*(Note : Dans RViz2, ajoutez un affichage "PoseArray" et écoutez le topic `/swarm_poses`).*
