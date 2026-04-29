import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import math

class MonteCarloLocalization(Node):
    def __init__(self):
        super().__init__('mcl_particle_filter')
        
        # Publisher pour afficher le nuage de particules (RViz2)
        self.particle_pub = self.create_publisher(PoseArray, '/particles', 10)
        
        # Paramètres du filtre
        self.num_particles = 500
        
        # État des particules: Tableau 2D [x, y, theta, poids]
        self.particles = np.zeros((self.num_particles, 4))
        self.initialize_particles_uniformly()
        
        # Bruit du modèle (incertitude mécanique)
        self.noise_v = 0.2
        self.noise_w = 0.05
        
        # Boucle principale (1 Hz) simulant le cycle de localisation
        self.timer = self.create_timer(1.0, self.mcl_loop)
        self.get_logger().info(f"Monte Carlo Localization started. Tracking {self.num_particles} parallel universes...")

    def initialize_particles_uniformly(self):
        """Disperse les particules aléatoirement sur une carte virtuelle de 10x10m"""
        self.particles[:, 0] = np.random.uniform(0, 10, self.num_particles) # X
        self.particles[:, 1] = np.random.uniform(0, 10, self.num_particles) # Y
        self.particles[:, 2] = np.random.uniform(-math.pi, math.pi, self.num_particles) # Theta
        self.particles[:, 3] = 1.0 / self.num_particles # Poids initial égal pour toutes

    def predict_motion(self, dt=1.0):
        """Étape 1: Modèle cinématique avec ajout de bruit gaussien"""
        # Simulation d'une commande: Le robot avance (v=0.5 m/s) et tourne (w=0.1 rad/s)
        v = 0.5
        w = 0.1
        
        # Application du déplacement à TOUTES les particules en une seule opération matricielle
        self.particles[:, 0] += v * dt * np.cos(self.particles[:, 2]) + np.random.normal(0, self.noise_v, self.num_particles)
        self.particles[:, 1] += v * dt * np.sin(self.particles[:, 2]) + np.random.normal(0, self.noise_v, self.num_particles)
        self.particles[:, 2] += w * dt + np.random.normal(0, self.noise_w, self.num_particles)

    def update_sensor_weights(self):
        """Étape 2: Modèle d'observation (Comparaison mesure réelle vs particules)"""
        # On simule un repère (landmark) situé aux coordonnées (5.0, 5.0)
        landmark = np.array([5.0, 5.0])
        
        # On simule la mesure du capteur : Le robot "réel" est à 2 mètres du repère
        measured_distance = 2.0
        sensor_noise = 0.5
        
        # Calcul de la probabilité de chaque particule via une distribution Gaussienne
        for i in range(self.num_particles):
            # Distance théorique si la particule était le vrai robot
            expected_dist = math.sqrt((self.particles[i, 0] - landmark[0])**2 + (self.particles[i, 1] - landmark[1])**2)
            
            # Mise à jour du poids: Plus la distance théorique est proche de la mesure, plus le poids est élevé
            self.particles[i, 3] = math.exp(-((expected_dist - measured_distance)**2) / (2 * sensor_noise**2))
            
        # Normalisation des poids (la somme doit valoir 1)
        total_weight = np.sum(self.particles[:, 3])
        if total_weight > 0:
            self.particles[:, 3] /= total_weight
        else:
            self.initialize_particles_uniformly() # Kidnapped robot problem fallback

    def systematic_resampling(self):
        """Étape 3: Rééchantillonnage (Darwinisme)"""
        # On tire au sort les particules qui survivent en fonction de leur poids
        indices = np.random.choice(self.num_particles, self.num_particles, p=self.particles[:, 3])
        
        # Remplacement de l'ancien nuage par les survivants clonés
        self.particles = self.particles[indices]
        
        # Réinitialisation des poids
        self.particles[:, 3] = 1.0 / self.num_particles

    def publish_pose_array(self):
        """Publication sur ROS 2 pour visualisation de la convergence"""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for i in range(self.num_particles):
            pose = Pose()
            pose.position.x = float(self.particles[i, 0])
            pose.position.y = float(self.particles[i, 1])
            # Conversion très simplifiée de l'angle pour Rviz
            pose.orientation.z = math.sin(self.particles[i, 2] / 2.0)
            pose.orientation.w = math.cos(self.particles[i, 2] / 2.0)
            msg.poses.append(pose)
            
        self.particle_pub.publish(msg)

    def mcl_loop(self):
        """Exécution séquentielle du filtre"""
        self.predict_motion()
        self.update_sensor_weights()
        self.systematic_resampling()
        self.publish_pose_array()
        self.get_logger().debug("Particle cloud updated and resampled.")

def main(args=None):
    rclpy.init(args=args)
    node = MonteCarloLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
