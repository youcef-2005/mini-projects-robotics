import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class TerminatorTracker(Node):
    def __init__(self):
        super().__init__('terminator_tracker')
        
        # Publisher pour bouger le robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscriber pour l'oeil du robot (Caméra)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        
        # Chargement de l'IA de détection (Haar Cascade pour visages)
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Paramètres du Contrôleur Proportionnel
        self.kp_angular = 0.005  # Sensibilité de la rotation
        self.kp_linear = 0.01    # Sensibilité de l'accélération
        self.target_size = 150   # Taille du visage désirée (plus c'est grand, plus il s'approche)

        self.get_logger().info("Terminator Mode Activated: Searching for human targets...")

    def image_callback(self, msg):
        # 1. Conversion de l'image ROS 2 en image lisible par OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        height, width, _ = frame.shape
        center_x = width // 2
        
        # 2. Détection de la cible (Visage humain)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        twist = Twist()
        
        if len(faces) > 0:
            # 3. Verrouillage de la première cible trouvée
            (x, y, w, h) = faces[0]
            target_center_x = x + w // 2
            
            # Calcul de l'erreur (Distance entre le centre de la caméra et la cible)
            error_x = center_x - target_center_x
            
            # Asservissement Angulaire : Tourner pour centrer la cible
            twist.angular.z = float(self.kp_angular * error_x)
            
            # Asservissement Linéaire : Avancer si la cible est trop loin
            error_size = self.target_size - w
            if w < self.target_size:
                twist.linear.x = float(self.kp_linear * error_size)
            
            self.get_logger().info(f"Target Locked! Error X: {error_x}. Intercepting...")
            
            # Optionnel : Dessiner le viseur Terminator sur l'image pour le debug
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 3)
            cv2.putText(frame, "TARGET LOCKED", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            
        else:
            # Si aucune cible, le robot tourne sur lui-même pour chercher
            twist.angular.z = 0.5
            self.get_logger().info("Searching...")
            
        self.cmd_pub.publish(twist)
        
        # Affichage du flux vidéo (HUD)
        cv2.imshow("Terminator HUD", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TerminatorTracker()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
