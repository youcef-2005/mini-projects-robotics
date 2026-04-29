import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class ExtendedKalmanFilterNode(Node):
    def __init__(self):
        super().__init__('ekf_sensor_fusion')
        
        # Publisher pour visualiser le résultat de la fusion de capteurs
        self.estimated_pub = self.create_publisher(PoseStamped, '/ekf/estimated_pose', 10)
        
        # --- MATRICES D'ÉTAT DU FILTRE DE KALMAN ---
        # Vecteur d'état estimé [x, y, theta]^T
        self.x_est = np.zeros((3, 1))
        
        # Matrice de covariance de l'estimation (P) - Incertitude initiale
        self.P_est = np.eye(3)
        
        # Matrice de covariance du bruit de processus (Q) - Confiance en l'odométrie
        # Erreur sur [x, y, theta] à chaque pas
        self.Q = np.diag([
            0.1,  # variance x
            0.1,  # variance y
            np.deg2rad(1.0) # variance theta
        ]) ** 2
        
        # Matrice de covariance du bruit de mesure (R) - Confiance au "GPS"
        # Le GPS simulé est très bruité (variance de 1.0 mètre)
        self.R = np.diag([1.0, 1.0]) ** 2
        
        # --- VARIABLES DE SIMULATION INTERNE ---
        self.x_true = np.zeros((3, 1)) # La vraie position secrète du robot
        self.x_dead_reckoning = np.zeros((3, 1)) # Position si on n'utilise que les roues (dérive)
        
        self.dt = 0.1 # Pas de temps (10 Hz)
        self.sim_time = 0.0
        self.max_time = 20.0 # Durée de la simulation en secondes
        
        self.timer = self.create_timer(self.dt, self.ekf_loop)
        self.get_logger().info("EKF Engine Started: Fusing noisy Odometry and GPS data in real-time...")

    def calculate_input_command(self):
        """Génère une commande de vitesse (le robot fait des 8)"""
        v = 1.0 # m/s
        yaw_rate = 0.5 * math.cos(self.sim_time / 2.0) # rad/s
        return np.array([[v], [yaw_rate]])

    def observation_model(self, x):
        """Ce que le GPS est censé voir (H_x)"""
        # Notre "GPS" virtuel ne voit que X et Y, il ne connaît pas l'angle (theta)
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        z = H @ x
        return z

    def jacobian_F(self, x, u):
        """Matrice Jacobienne F (Modèle de transition d'état dérivé)"""
        v = u[0, 0]
        theta = x[2, 0]
        
        # Dérivées partielles des équations de mouvement
        F = np.array([
            [1.0, 0.0, -self.dt * v * math.sin(theta)],
            [0.0, 1.0,  self.dt * v * math.cos(theta)],
            [0.0, 0.0, 1.0]
        ])
        return F

    def jacobian_H(self):
        """Matrice Jacobienne H (Modèle d'observation dérivé)"""
        # Comme le modèle d'observation est linéaire (juste X et Y), la Jacobienne est constante
        H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])
        return H

    def simulate_environment(self, u):
        """Génère le monde réel, les mesures bruitées et la dérive pure"""
        # 1. Mise à jour de la vraie position (Ground Truth)
        theta = self.x_true[2, 0]
        B = np.array([
            [self.dt * math.cos(theta), 0],
            [self.dt * math.sin(theta), 0],
            [0, self.dt]
        ])
        self.x_true = self.x_true + B @ u
        
        # 2. Simulation de l'odométrie bruitée (Dead Reckoning)
        # On ajoute du bruit artificiel aux commandes envoyées aux roues
        u_noisy = u + np.array([[np.random.randn() * 0.1], [np.random.randn() * 0.05]])
        theta_dr = self.x_dead_reckoning[2, 0]
        B_dr = np.array([
            [self.dt * math.cos(theta_dr), 0],
            [self.dt * math.sin(theta_dr), 0],
            [0, self.dt]
        ])
        self.x_dead_reckoning = self.x_dead_reckoning + B_dr @ u_noisy
        
        # 3. Simulation du capteur GPS (très bruité)
        z_true = self.observation_model(self.x_true)
        z_gps_noisy = z_true + np.array([[np.random.randn() * 1.0], [np.random.randn() * 1.0]])
        
        return u_noisy, z_gps_noisy

    def ekf_loop(self):
        """Cycle principal: Prédiction -> Mise à jour"""
        if self.sim_time >= self.max_time:
            # Calcul de l'erreur finale pour prouver l'efficacité
            err_dr = np.linalg.norm(self.x_true[0:2] - self.x_dead_reckoning[0:2])
            err_ekf = np.linalg.norm(self.x_true[0:2] - self.x_est[0:2])
            
            self.get_logger().info("--- SIMULATION FINISHED ---")
            self.get_logger().info(f"Final Drift Error (Odometry only): {err_dr:.2f} meters")
            self.get_logger().info(f"Final EKF Error (Sensor Fusion):   {err_ekf:.2f} meters")
            
            self.timer.cancel()
            return

        # Obtenir les commandes et les fausses mesures
        u_true = self.calculate_input_command()
        u_cmd, z_meas = self.simulate_environment(u_true)

        # ==========================================
        # ÉTAPE 1 : PRÉDICTION (Modèle Cinématique)
        # ==========================================
        theta_est = self.x_est[2, 0]
        B_est = np.array([
            [self.dt * math.cos(theta_est), 0],
            [self.dt * math.sin(theta_est), 0],
            [0, self.dt]
        ])
        
        # Prédiction de l'état
        x_pred = self.x_est + B_est @ u_cmd
        
        # Calcul de la Jacobienne F
        F = self.jacobian_F(self.x_est, u_cmd)
        
        # Prédiction de la covariance
        P_pred = F @ self.P_est @ F.T + self.Q

        # ==========================================
        # ÉTAPE 2 : MISE À JOUR (Correction avec le GPS)
        # ==========================================
        H = self.jacobian_H()
        
        # Innovation (Erreur entre ce que le GPS dit et ce qu'on prévoyait)
        z_pred = self.observation_model(x_pred)
        y = z_meas - z_pred
        
        # Innovation Covariance (S)
        S = H @ P_pred @ H.T + self.R
        
        # Gain de Kalman (K) - Détermine à qui on fait le plus confiance
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # Mise à jour finale de l'état estimé
        self.x_est = x_pred + K @ y
        
        # Mise à jour finale de la covariance
        I = np.eye(self.x_est.shape[0])
        self.P_est = (I - K @ H) @ P_pred

        # --- Publication du résultat ---
        self.publish_estimated_pose()
        
        self.sim_time += self.dt
        
        if int(self.sim_time * 10) % 10 == 0:
             self.get_logger().debug(f"Time: {self.sim_time:.1f}s | Kalman Gain matrix computed.")

    def publish_estimated_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self.x_est[0, 0])
        msg.pose.position.y = float(self.x_est[1, 0])
        # Simple quaternion conversion for 2D heading
        msg.pose.orientation.z = math.sin(self.x_est[2, 0] / 2.0)
        msg.pose.orientation.w = math.cos(self.x_est[2, 0] / 2.0)
        self.estimated_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ExtendedKalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
