import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import numpy as np
import math

class EKFSensorFusion(Node):
    def __init__(self):
        super().__init__('ekf_sensor_fusion')
        
        # Subscriptions: Odom (bruitée) et IMU (bruitée)
        self.odom_sub = self.create_subscription(Odometry, '/odom_noisy', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Publisher: L'odométrie fusionnée et propre
        self.fused_pub = self.create_publisher(Odometry, '/odom_fused', 10)
        
        # Initialisation du vecteur d'état [x, y, theta, v, w]^T
        self.X = np.zeros((5, 1))
        
        # Matrice de covariance de l'erreur (P)
        self.P = np.eye(5) * 0.1
        
        # Bruit du modèle (Q) - incertitude de la cinématique
        self.Q = np.diag([0.05, 0.05, 0.01, 0.1, 0.01])
        
        # Bruit de mesure (R) - incertitude des capteurs
        self.R = np.diag([0.2, 0.2, 0.05]) # On mesure x, y via odom et theta via IMU
        
        self.last_time = self.get_clock().now()
        self.latest_imu_yaw = 0.0
        
        self.get_logger().info("Extended Kalman Filter (EKF) Node Initialized. Fusing Odometry and IMU...")

    def quaternion_to_yaw(self, q):
        # Conversion simple d'un quaternion en angle de lacet (yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        q = [0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)]
        return q

    def imu_callback(self, msg):
        # Mise à jour asynchrone de l'orientation via la centrale inertielle
        self.latest_imu_yaw = self.quaternion_to_yaw(msg.orientation)

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # 1. ÉTAPE DE PRÉDICTION (Modèle Cinématique)
        x, y, theta, v, w = self.X.flatten()
        
        # Prédiction de l'état suivant
        X_pred = np.array([
            [x + v * dt * math.cos(theta)],
            [y + v * dt * math.sin(theta)],
            [theta + w * dt],
            [v],
            [w]
        ])
        
        # Jacobienne du modèle cinématique (Matrice F)
        F = np.array([
            [1, 0, -v * dt * math.sin(theta), dt * math.cos(theta), 0],
            [0, 1,  v * dt * math.cos(theta), dt * math.sin(theta), 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        
        # Mise à jour de la covariance (P = F*P*F^T + Q)
        P_pred = F @ self.P @ F.T + self.Q
        
        # 2. ÉTAPE DE MISE À JOUR (Correction avec les mesures)
        # Vecteur de mesure Z [x_odom, y_odom, theta_imu]^T
        Z = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [self.latest_imu_yaw]
        ])
        
        # Matrice d'observation (H) - Mappe l'état vers les mesures
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ])
        
        # Innovation (Y = Z - H*X_pred)
        Y = Z - (H @ X_pred)
        # Normalisation de l'angle entre -pi et pi
        Y[2, 0] = math.atan2(math.sin(Y[2, 0]), math.cos(Y[2, 0]))
        
        # Gain de Kalman (K = P*H^T * (H*P*H^T + R)^-1)
        S = H @ P_pred @ H.T + self.R
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # Mise à jour de l'état final et de la covariance
        self.X = X_pred + (K @ Y)
        self.P = (np.eye(5) - K @ H) @ P_pred
        
        # 3. PUBLICATION DU RÉSULTAT FUSIONNÉ
        fused_msg = Odometry()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = "odom"
        fused_msg.child_frame_id = "base_link"
        
        fused_msg.pose.pose.position.x = float(self.X[0, 0])
        fused_msg.pose.pose.position.y = float(self.X[1, 0])
        
        q = self.yaw_to_quaternion(float(self.X[2, 0]))
        fused_msg.pose.pose.orientation.x = q[0]
        fused_msg.pose.pose.orientation.y = q[1]
        fused_msg.pose.pose.orientation.z = q[2]
        fused_msg.pose.pose.orientation.w = q[3]
        
        fused_msg.twist.twist.linear.x = float(self.X[3, 0])
        fused_msg.twist.twist.angular.z = float(self.X[4, 0])
        
        self.fused_pub.publish(fused_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSensorFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
