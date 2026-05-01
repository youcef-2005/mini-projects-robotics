import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import math

class DecentralizedSwarm(Node):
    def __init__(self):
        super().__init__('swarm_flocking')
        
        # Publisher pour afficher les 50 robots d'un coup dans RViz
        self.swarm_pub = self.create_publisher(PoseArray, '/swarm_poses', 10)
        
        # --- CONFIGURATION DE L'ESSAIM ---
        self.num_agents = 50
        self.bounds = 30.0 # Arène de 30x30 mètres
        
        # État global (Matrices Nx2 pour X et Y)
        # Positions initiales aléatoires
        self.pos = np.random.uniform(-self.bounds/2, self.bounds/2, (self.num_agents, 2))
        # Vitesses initiales aléatoires
        self.vel = np.random.uniform(-1.0, 1.0, (self.num_agents, 2))
        
        # --- HYPERPARAMÈTRES DU COMPORTEMENT ÉMERGENT ---
        self.visual_range = 4.0      # Rayon de vision du robot
        self.protected_range = 1.0   # Rayon critique anticollision
        
        self.k_separation = 0.08     # Force de répulsion (éviter les crashs)
        self.k_alignment = 0.05      # Force d'alignement (copier la direction des voisins)
        self.k_cohesion = 0.01       # Force de cohésion (rester en groupe)
        self.turn_factor = 0.5       # Force de rebond sur les murs invisibles
        
        self.max_speed = 3.0
        self.min_speed = 1.0
        
        self.dt = 0.05 # Simulation à 20 Hz
        self.timer = self.create_timer(self.dt, self.update_swarm_physics)
        
        self.get_logger().info(f"🌐 Swarm Engine Activated: Computing decentralized matrices for {self.num_agents} autonomous agents...")

    def update_swarm_physics(self):
        """Moteur de calcul vectoriel pour O(N^2) interactions"""
        
        # 1. Calcul de la matrice des distances (Chaque robot vers chaque robot)
        # diff shape: (N, N, 2), dist shape: (N, N)
        diff = self.pos[:, np.newaxis, :] - self.pos[np.newaxis, :, :]
        dist = np.linalg.norm(diff, axis=-1)
        
        # Initialisation des vecteurs de force pour cette itération
        separation = np.zeros((self.num_agents, 2))
        alignment = np.zeros((self.num_agents, 2))
        cohesion = np.zeros((self.num_agents, 2))
        
        for i in range(self.num_agents):
            # --- RÈGLE 1 : SÉPARATION (Anti-collision) ---
            # Trouve les voisins trop proches
            close_mask = (dist[i] < self.protected_range) & (dist[i] > 0)
            if np.any(close_mask):
                # S'éloigner du barycentre des robots trop proches
                separation[i] = np.sum(self.pos[i] - self.pos[close_mask], axis=0) * self.k_separation
            
            # --- RÈGLES 2 & 3 : ALIGNEMENT ET COHÉSION ---
            # Trouve les voisins dans le champ de vision (mais pas trop proches)
            visible_mask = (dist[i] < self.visual_range) & (dist[i] >= self.protected_range)
            if np.any(visible_mask):
                # Alignement : Ajuster sa vitesse sur la moyenne des voisins
                avg_vel = np.mean(self.vel[visible_mask], axis=0)
                alignment[i] = (avg_vel - self.vel[i]) * self.k_alignment
                
                # Cohésion : Se diriger vers le centre de gravité des voisins
                avg_pos = np.mean(self.pos[visible_mask], axis=0)
                cohesion[i] = (avg_pos - self.pos[i]) * self.k_cohesion
                
            # --- RÈGLE 4 : LIMITES DE L'ARÈNE (Mur de force) ---
            if self.pos[i, 0] > self.bounds/2: self.vel[i, 0] -= self.turn_factor
            if self.pos[i, 0] < -self.bounds/2: self.vel[i, 0] += self.turn_factor
            if self.pos[i, 1] > self.bounds/2: self.vel[i, 1] -= self.turn_factor
            if self.pos[i, 1] < -self.bounds/2: self.vel[i, 1] += self.turn_factor

        # 2. Intégration des forces (Mise à jour des vitesses)
        self.vel += separation + alignment + cohesion
        
        # 3. Saturation (Régulation mathématique de la vitesse min/max)
        speeds = np.linalg.norm(self.vel, axis=-1)
        speeds[speeds == 0] = 0.001 # Éviter la division par zéro
        
        too_fast = speeds > self.max_speed
        self.vel[too_fast] = (self.vel[too_fast] / speeds[too_fast, np.newaxis]) * self.max_speed
        
        too_slow = speeds < self.min_speed
        self.vel[too_slow] = (self.vel[too_slow] / speeds[too_slow, np.newaxis]) * self.min_speed
        
        # 4. Cinématique (Mise à jour des positions)
        self.pos += self.vel * self.dt
        
        # 5. Publication de l'état global pour la visualisation
        self.publish_swarm_state()

    def publish_swarm_state(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for i in range(self.num_agents):
            p = Pose()
            p.position.x = float(self.pos[i, 0])
            p.position.y = float(self.pos[i, 1])
            p.position.z = 0.0
            
            # Calcul de l'orientation (Yaw) en fonction du vecteur de vitesse
            theta = math.atan2(self.vel[i, 1], self.vel[i, 0])
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = math.sin(theta / 2.0)
            p.orientation.w = math.cos(theta / 2.0)
            
            msg.poses.append(p)
            
        self.swarm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DecentralizedSwarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
