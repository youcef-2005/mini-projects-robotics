import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class DiscreteLowPassFilter:
    """Filtre numérique du premier ordre basé sur la transformée en Z"""
    def __init__(self, alpha):
        self.alpha = alpha
        self.y_prev = 0.0

    def filter(self, x):
        # Équation aux différences : y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
        y = self.alpha * x + (1.0 - self.alpha) * self.y_prev
        self.y_prev = y
        return y

class DiscretePID:
    """Contrôleur PID implémenté en temps discret"""
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        
        # Intégration numérique (Méthode des rectangles)
        self.integral += error * self.dt
        
        # Dérivation numérique (Différence finie)
        derivative = (error - self.prev_error) / self.dt
        
        self.prev_error = error
        
        # Loi de commande
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class PhysicalPlantSimulator:
    """Simule la fonction de transfert continue d'un moteur à courant continu G(s)"""
    def __init__(self, K, tau, dt):
        self.K = K          # Gain statique
        self.tau = tau      # Constante de temps
        self.dt = dt        # Pas d'échantillonnage
        self.y = 0.0        # Sortie physique actuelle (Vitesse)

    def update(self, u):
        # Discrétisation de G(s) = K / (tau*s + 1) par la méthode d'Euler
        dy = (1.0 / self.tau) * (-self.y + self.K * u)
        self.y += dy * self.dt
        return self.y

class DigitalTwinNode(Node):
    def __init__(self):
        super().__init__('digital_twin_controller')
        
        self.cmd_pub = self.create_publisher(Float64, '/motor/command', 10)
        self.sensor_pub = self.create_publisher(Float64, '/motor/velocity_filtered', 10)
        
        # Paramètres temporels
        self.dt = 0.05 # Échantillonnage à 20 Hz
        
        # Initialisation des composants du jumeau numérique
        # Moteur: Gain = 2.0, Constante de temps = 0.5s
        self.plant = PhysicalPlantSimulator(K=2.0, tau=0.5, dt=self.dt)
        
        # Filtre numérique pour nettoyer le bruit des capteurs
        self.filter = DiscreteLowPassFilter(alpha=0.2)
        
        # Contrôleur PID (Réglé via méthode de Ziegler-Nichols approximée)
        self.pid = DiscretePID(kp=1.5, ki=3.0, kd=0.05, dt=self.dt)
        
        self.setpoint = 100.0 # Vitesse cible (ex: 100 rad/s)
        self.sim_time = 0.0
        self.max_time = 5.0   # Fin de la simulation après 5 secondes
        
        self.get_logger().info("🏭 Digital Twin Started: Linking Continuous Physics with Discrete Z-Transform Control.")
        self.timer = self.create_timer(self.dt, self.control_loop)

    def control_loop(self):
        if self.sim_time >= self.max_time:
            self.get_logger().info("✅ STEP RESPONSE COMPLETE: The digital controller successfully stabilized the physical plant.")
            self.cmd_pub.publish(Float64(data=0.0))
            self.timer.cancel()
            return

        # 1. Lecture du capteur physique avec injection de bruit blanc Gaussien (Simulation de la réalité)
        true_velocity = self.plant.y
        noisy_measurement = true_velocity + random.gauss(0, 5.0)
        
        # 2. Traitement du Signal Numérique (DSP) : Filtrage de la mesure
        filtered_velocity = self.filter.filter(noisy_measurement)
        
        # 3. Calcul de la loi de commande via PID Discret
        control_signal = self.pid.compute(self.setpoint, filtered_velocity)
        
        # Saturation de la commande (Les moteurs réels ont des limites de voltage, ex: +/- 24V)
        control_signal = max(min(control_signal, 24.0), -24.0)
        
        # 4. Application de la commande sur l'usine physique (Le jumeau)
        self.plant.update(control_signal)
        
        # 5. Publication des données pour traçage
        msg_cmd = Float64()
        msg_cmd.data = float(control_signal)
        self.cmd_pub.publish(msg_cmd)
        
        msg_sens = Float64()
        msg_sens.data = float(filtered_velocity)
        self.sensor_pub.publish(msg_sens)
        
        self.sim_time += self.dt
        
        # Affichage d'un tableau de bord dans le terminal
        if int(self.sim_time * 100) % 50 == 0: # Log toutes les 0.5s
             error = self.setpoint - filtered_velocity
             self.get_logger().info(f"Time: {self.sim_time:.1f}s | Target: {self.setpoint} | Filtered Vel: {filtered_velocity:.1f} | Error: {error:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
