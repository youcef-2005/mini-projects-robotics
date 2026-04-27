import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class DistanceController(Node):
    def __init__(self):
        super().__init__('distance_controller')
        
        # Publisher pour envoyer les commandes de vitesse
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber pour lire la position du robot
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.target_distance = 50.0  # Mètres
        self.start_x = None
        self.start_y = None
        self.distance_traveled = 0.0
        self.goal_reached = False

        self.get_logger().info(f"Démarrage de la mission : Avancer de {self.target_distance} mètres.")

    def odom_callback(self, msg):
        if self.goal_reached:
            return

        # Récupération de la position actuelle (x, y)
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Initialisation du point de départ au premier message reçu
        if self.start_x is None and self.start_y is None:
            self.start_x = current_x
            self.start_y = current_y
            self.get_logger().info(f"Position initiale enregistrée : X={self.start_x:.2f}, Y={self.start_y:.2f}")

        # Calcul de la distance euclidienne parcourue
        self.distance_traveled = math.sqrt(
            (current_x - self.start_x)**2 + (current_y - self.start_y)**2
        )

        # Logique de contrôle
        twist_msg = Twist()
        if self.distance_traveled < self.target_distance:
            twist_msg.linear.x = 0.8  # Vitesse de 0.8 m/s
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info(f"Progression : {self.distance_traveled:.2f} / {self.target_distance} m")
        else:
            twist_msg.linear.x = 0.0  # Arrêt d'urgence/freinage
            self.cmd_vel_pub.publish(twist_msg)
            self.goal_reached = True
            self.get_logger().info("Cible atteinte ! Arrêt des moteurs.")

def main(args=None):
    rclpy.init(args=args)
    node = DistanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
