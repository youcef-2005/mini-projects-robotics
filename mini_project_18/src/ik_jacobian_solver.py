import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class InverseKinematicsSolver(Node):
    def __init__(self):
        super().__init__('ik_jacobian_solver')
        
        # Publisher pour envoyer les angles articulaires calculés (lisible par RViz)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # --- PARAMÈTRES DU BRAS MANIPULATEUR (Architecture 4-DOF RRRR) ---
        # Longueur des segments en mètres (Base, Épaule, Coude, Poignet)
        self.links = [0.4, 0.3, 0.3, 0.2]
        
        # État articulaire actuel [q1, q2, q3, q4] en radians
        self.q = np.array([0.0, 0.5, -0.5, 0.0])
        
        # --- CIBLE SPATIALE (La position x, y, z que le bras doit atteindre) ---
        self.target_pos = np.array([0.5, 0.5, 0.6])
        
        # Paramètres du solveur
        self.epsilon = 1e-4  # Delta pour le calcul numérique de la dérivée
        self.alpha = 0.1     # Taux d'apprentissage (vitesse de convergence)
        self.tolerance = 0.005 # Précision millimétrique (5 mm)
        
        self.mission_accomplished = False
        
        # Boucle d'optimisation (Tourne à 10 Hz)
        self.timer = self.create_timer(0.1, self.ik_optimization_loop)
        self.get_logger().info("Iterative IK Solver Initialized. Calculating Jacobian for 4-DOF RRRR arm...")

    def forward_kinematics(self, q):
        """Calcule la position (x,y,z) de l'effecteur en fonction des angles articulaires"""
        l1, l2, l3, l4 = self.links
        q1, q2, q3, q4 = q
        
        # Modèle géométrique direct (RRRR 3D)
        # q1: Rotation de la base autour de l'axe Z
        # q2, q3, q4: Rotations d'élévation autour de l'axe Y
        
        r = l2 * np.cos(q2) + l3 * np.cos(q2 + q3) + l4 * np.cos(q2 + q3 + q4)
        
        x = r * np.cos(q1)
        y = r * np.sin(q1)
        z = l1 + l2 * np.sin(q2) + l3 * np.sin(q2 + q3) + l4 * np.sin(q2 + q3 + q4)
        
        return np.array([x, y, z])

    def compute_jacobian(self, q):
        """Calcule la Matrice Jacobienne (3x4) par différenciation numérique"""
        J = np.zeros((3, 4))
        current_pos = self.forward_kinematics(q)
        
        for i in range(4):
            # On perturbe très légèrement une articulation
            q_perturbed = q.copy()
            q_perturbed[i] += self.epsilon
            
            # On regarde l'impact sur la position finale (x, y, z)
            perturbed_pos = self.forward_kinematics(q_perturbed)
            
            # Dérivée partielle : (f(x+h) - f(x)) / h
            J[:, i] = (perturbed_pos - current_pos) / self.epsilon
            
        return J

    def ik_optimization_loop(self):
        """Le cœur de l'algorithme : descente de gradient via la Pseudo-Inverse"""
        if self.mission_accomplished:
            return

        # 1. Calcul de l'erreur spatiale (Où on est vs. Où on veut aller)
        current_pos = self.forward_kinematics(self.q)
        error = self.target_pos - current_pos
        distance = np.linalg.norm(error)
        
        # 2. Condition de succès
        if distance < self.tolerance:
            self.get_logger().info(f"TARGET REACHED! Final Error: {distance*1000:.2f} mm")
            self.mission_accomplished = True
            return

        # 3. Calcul de la Matrice Jacobienne
        J = self.compute_jacobian(self.q)
        
        # 4. Calcul de la Pseudo-Inverse de Moore-Penrose (gère les singularités)
        J_pinv = np.linalg.pinv(J)
        
        # 5. Calcul de l'incrément articulaire (Delta Q = J^+ * Error)
        delta_q = np.dot(J_pinv, error)
        
        # 6. Mise à jour des angles articulaires
        self.q = self.q + self.alpha * delta_q
        
        # 7. Publication des joints pour visualisation
        self.publish_joints()
        
        self.get_logger().debug(f"Distance to target: {distance:.3f} m")

    def publish_joints(self):
        """Formate et publie le message ROS 2"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1_base', 'joint2_shoulder', 'joint3_elbow', 'joint4_wrist']
        msg.position = self.q.tolist()
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
