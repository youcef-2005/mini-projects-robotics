import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- CONFIGURATION DU MPC ---
        self.horizon = 10        # Nombre d'étapes prédites dans le futur
        self.dt = 0.1            # Pas de temps (sec)
        
        # État actuel du robot [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        
        # Trajectoire de référence (Une ligne droite puis un virage)
        self.ref_path = self.generate_reference_path()
        
        # Coûts de l'optimisation (Poids)
        self.q_pos = 10.0        # Importance de suivre la position
        self.q_theta = 1.0       # Importance de suivre l'orientation
        self.r_v = 0.1           # Coût de l'effort (vitesse)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("MPC Controller Online: Predicting future states...")

    def generate_reference_path(self):
        """Génère une trajectoire sinusoïdale de référence"""
        path = []
        for i in range(200):
            t = i * self.dt
            x = 1.0 * t
            y = 2.0 * math.sin(0.5 * x)
            path.append([x, y])
        return np.array(path)

    def predict_state(self, current_state, v, w, dt):
        """Modèle cinématique unicycle pour la prédiction"""
        x, y, theta = current_state
        new_x = x + v * math.cos(theta) * dt
        new_y = y + v * math.sin(theta) * dt
        new_theta = theta + w * dt
        return np.array([new_x, new_y, new_theta])

    def control_loop(self):
        """Le cœur du MPC : Optimisation par recherche itérative"""
        # Trouver l'index le plus proche sur la trajectoire de référence
        dists = np.linalg.norm(self.ref_path - self.state[:2], axis=1)
        idx = np.argmin(dists)
        
        best_v = 0.0
        best_w = 0.0
        min_cost = float('inf')
        
        # Échantillonnage de l'espace de commande (Solveur itératif robuste)
        # On teste plusieurs combinaisons de vitesses pour trouver l'optimale
        for v_test in np.linspace(0.0, 1.0, 5):
            for w_test in np.linspace(-1.0, 1.0, 9):
                
                # Simuler le futur sur l'horizon
                temp_state = self.state.copy()
                total_cost = 0.0
                
                for h in range(self.horizon):
                    temp_state = self.predict_state(temp_state, v_test, w_test, self.dt)
                    
                    # Calcul du coût par rapport à la référence future
                    ref_idx = min(idx + h, len(self.ref_path) - 1)
                    error_pos = np.linalg.norm(temp_state[:2] - self.ref_path[ref_idx])
                    
                    total_cost += self.q_pos * (error_pos**2)
                
                # Ajout d'un coût sur la commande (pour la fluidité)
                total_cost += self.r_v * (v_test**2)

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_v = v_test
                    best_w = w_test

        # Appliquer la meilleure commande trouvée
        twist = Twist()
        twist.linear.x = float(best_v)
        twist.angular.z = float(best_w)
        self.cmd_pub.publish(twist)
        
        # Mise à jour de la simulation interne (pour que ça marche sans robot)
        self.state = self.predict_state(self.state, best_v, best_w, self.dt)
        
        if idx >= len(self.ref_path) - 5:
            self.get_logger().info("Trajectoire terminée avec succès !")
            self.cmd_pub.publish(Twist())
            self.timer.cancel()

def main():
    rclpy.init()
    node = MPCController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
