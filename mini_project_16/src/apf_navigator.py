import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math

class APFNavigator(Node):
    def __init__(self):
        super().__init__('apf_navigator')
        
        # Publisher pour les commandes de vitesse
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- PARAMÈTRES DE LA SIMULATION INTERNE ---
        # Position initiale du robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Position de la cible (Goal)
        self.goal_x = 10.0
        self.goal_y = 10.0
        
        # Liste des obstacles (x, y)
        self.obstacles = np.array([
            [4.0, 3.0],
            [5.0, 6.0],
            [7.0, 8.0],
            [2.0, 5.0]
        ])
        
        # --- PARAMÈTRES DU CHAMP DE POTENTIEL ---
        self.k_att = 1.5      # Gain d'attraction vers la cible
        self.k_rep = 5.0      # Gain de répulsion des obstacles
        self.d_obs = 3.0      # Distance d'influence (le robot ignore l'obstacle s'il est plus loin)
        
        # --- PARAMÈTRES DU CONTRÔLEUR ---
        self.max_v = 1.0      # Vitesse linéaire max (m/s)
        self.max_w = 1.5      # Vitesse angulaire max (rad/s)
        self.k_w = 2.0        # Gain proportionnel pour la rotation
        
        self.dt = 0.1         # Pas de temps de la simulation (10 Hz)
        self.mission_accomplished = False
        
        # Boucle de contrôle à 10 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("APF Navigation started: Calculating force vectors...")

    def calculate_attractive_force(self):
        """Calcule le vecteur force qui attire le robot vers la cible"""
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist = math.hypot(dx, dy)
        
        # Force d'attraction proportionnelle à la distance
        f_x = self.k_att * dx
        f_y = self.k_att * dy
        return f_x, f_y, dist

    def calculate_repulsive_force(self):
        """Calcule la somme des vecteurs de répulsion générés par les obstacles"""
        f_x = 0.0
        f_y = 0.0
        
        for obs in self.obstacles:
            dx = self.robot_x - obs[0]
            dy = self.robot_y - obs[1]
            dist = math.hypot(dx, dy)
            
            # L'obstacle ne repousse le robot que s'il est assez proche
            if dist < self.d_obs and dist > 0.01:
                # Formule de répulsion de Khatib
                force = self.k_rep * (1.0/dist - 1.0/self.d_obs) * (1.0/(dist**2))
                f_x += force * (dx / dist)
                f_y += force * (dy / dist)
                
        return f_x, f_y

    def control_loop(self):
        """Boucle principale : calcule les forces, déplace le robot virtuel et publie la vitesse"""
        if self.mission_accomplished:
            return

        # 1. Calcul des forces (Attraction + Répulsion)
        f_att_x, f_att_y, dist_to_goal = self.calculate_attractive_force()
        f_rep_x, f_rep_y = self.calculate_repulsive_force()
        
        # Force résultante totale
        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y
        
        # 2. Condition de victoire
        if dist_to_goal < 0.2:
            self.get_logger().info("GOAL REACHED! Mission Accomplished.")
            self.cmd_pub.publish(Twist()) # Stop le robot
            self.mission_accomplished = True
            return

        # 3. Conversion du vecteur force en commandes de vitesse (Cinématique)
        # L'angle vers lequel la force veut envoyer le robot
        target_theta = math.atan2(f_total_y, f_total_x)
        
        # Erreur d'angle (normalisée entre -pi et pi)
        error_theta = target_theta - self.robot_theta
        error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
        
        # Calcul des commandes Twist
        twist = Twist()
        
        # Vitesse angulaire proportionnelle à l'erreur d'angle
        twist.angular.z = np.clip(self.k_w * error_theta, -self.max_w, self.max_w)
        
        # Vitesse linéaire proportionnelle à la magnitude de la force (réduite si on tourne beaucoup)
        force_magnitude = math.hypot(f_total_x, f_total_y)
        v_target = min(force_magnitude * 0.1, self.max_v)
        # On ralentit linéairement si on a un grand virage à faire
        twist.linear.x = max(0.0, v_target * math.cos(error_theta))
        
        # 4. Publication de la commande réelle
        self.cmd_pub.publish(twist)
        
        # 5. Simulation du déplacement interne (pour garantir que le code tourne seul)
        self.robot_theta += twist.angular.z * self.dt
        self.robot_x += twist.linear.x * math.cos(self.robot_theta) * self.dt
        self.robot_y += twist.linear.x * math.sin(self.robot_theta) * self.dt
        
        # Log de progression tous les pas (simplifié pour la démo)
        self.get_logger().debug(f"Pose: X={self.robot_x:.1f}, Y={self.robot_y:.1f} | Dist: {dist_to_goal:.1f}m")

def main(args=None):
    rclpy.init(args=args)
    node = APFNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
