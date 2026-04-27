import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class RRRRKinematicsSolver(Node):
    def __init__(self):
        super().__init__('rrrr_kinematics_solver')
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        
        # Paramètres géométriques du robot (Hauteur totale ~ 1.20m)
        self.L1 = 0.50  # Base au coude 1
        self.L2 = 0.40  # Coude 1 au coude 2
        self.L3 = 0.20  # Coude 2 au poignet
        self.L4 = 0.10  # Poignet à l'effecteur
        
        self.get_logger().info("4-DOF RRRR Kinematics Solver Initialized.")

    def dh_matrix(self, theta, d, a, alpha):
        """Génère la matrice de transformation homogène de Denavit-Hartenberg"""
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0,               math.sin(alpha),                  math.cos(alpha),                 d],
            [0,               0,                                0,                               1]
        ])

    def joint_callback(self, msg):
        try:
            # Récupération des 4 angles (thêta) du bras RRRR
            t1, t2, t3, t4 = msg.position[0:4]
            
            # Paramètres DH (theta, d, a, alpha)
            T01 = self.dh_matrix(t1, self.L1, 0, math.pi/2)
            T12 = self.dh_matrix(t2, 0, self.L2, 0)
            T23 = self.dh_matrix(t3, 0, self.L3, 0)
            T34 = self.dh_matrix(t4, 0, self.L4, 0)
            
            # Multiplication matricielle pour la pose finale : T04 = T01 * T12 * T23 * T34
            T04 = T01 @ T12 @ T23 @ T34
            
            # Extraction des coordonnées X, Y, Z
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = float(T04[0, 3])
            pose_msg.pose.position.y = float(T04[1, 3])
            pose_msg.pose.position.z = float(T04[2, 3])
            
            self.pose_pub.publish(pose_msg)
            self.get_logger().info(f"End Effector Pose: X={pose_msg.pose.position.x:.2f}, Y={pose_msg.pose.position.y:.2f}, Z={pose_msg.pose.position.z:.2f}")
            
        except IndexError:
            self.get_logger().warn("Waiting for exactly 4 joint states (RRRR configuration).")

def main(args=None):
    rclpy.init(args=args)
    node = RRRRKinematicsSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
