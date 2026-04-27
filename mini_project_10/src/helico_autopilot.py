import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class HelicoAutopilot(Node):
    def __init__(self):
        super().__init__('helico_autopilot')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Paramètres de mission (10 km)
        self.target_distance = 10000.0 
        self.target_altitude = 50.0    # Vol à 50m de hauteur
        self.start_pos = None
        self.mission_completed = False

        self.get_logger().info("Mission 10km : Autopilote activé. Phase de décollage...")

    def odom_callback(self, msg):
        if self.mission_completed:
            return

        curr = msg.pose.pose.position
        if self.start_pos is None:
            self.start_pos = (curr.x, curr.y)
            return

        # Calcul de la distance parcourue (Plan XY)
        dist = math.sqrt((curr.x - self.start_pos[0])**2 + (curr.y - self.start_pos[1])**2)
        
        twist = Twist()

        # 1. Gestion de l'altitude (Z)
        if curr.z < self.target_altitude - 0.5:
            twist.linear.z = 2.0  # Montée
        elif curr.z > self.target_altitude + 0.5:
            twist.linear.z = -1.0 # Descente corrective
        else:
            twist.linear.z = 0.0  # Altitude stabilisée

        # 2. Gestion de la progression (X)
        if dist < self.target_distance:
            twist.linear.x = 25.0  # Vitesse de croisière (~90 km/h)
            if dist % 1000 < 5: # Log tous les kilomètres
                self.get_logger().info(f"Distance : {dist/1000:.1f} km parcourus...")
        else:
            # 3. Phase d'arrivée
            twist.linear.x = 0.0
            if curr.z > 0.5:
                twist.linear.z = -1.5 # Atterrissage
            else:
                self.mission_completed = True
                self.get_logger().info("Mission 10km terminée. Atterrissage réussi.")

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = HelicoAutopilot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
