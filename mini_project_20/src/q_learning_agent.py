import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import random

class QLearningNavigator(Node):
    def __init__(self):
        super().__init__('q_learning_agent')
        
        # Publisher pour afficher la position du robot une fois entraîné
        self.pose_pub = self.create_publisher(Point, '/ai_robot_pose', 10)
        
        # --- ENVIRONNEMENT (Le Labyrinthe Virtuel) ---
        self.grid_size = 6
        self.start_state = (0, 0)
        self.goal_state = (5, 5)
        # Murs que le robot doit apprendre à éviter
        self.obstacles = [(1, 1), (1, 2), (2, 2), (3, 4), (4, 1), (4, 2)]
        
        # --- HYPERPARAMÈTRES DE L'IA (Q-Learning) ---
        self.alpha = 0.1          # Taux d'apprentissage (Vitesse à laquelle il apprend)
        self.gamma = 0.9          # Facteur d'escompte (Importance des récompenses futures)
        self.epsilon = 1.0        # Taux d'exploration initial (100% au début)
        self.epsilon_min = 0.01   # Taux d'exploration minimal
        self.epsilon_decay = 0.99 # Diminution progressive de l'exploration
        self.episodes = 600       # Nombre de vies/essais pour apprendre
        
        # Actions: 0:Haut, 1:Bas, 2:Gauche, 3:Droite
        self.actions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        # Initialisation de la Q-Table (La "mémoire" du robot, remplie de zéros au départ)
        self.q_table = np.zeros((self.grid_size, self.grid_size, len(self.actions)))
        
        self.get_logger().info("🟢 INIT: Booting Reinforcement Learning Engine...")
        self.get_logger().info(f"🧠 TRAINING: Simulating {self.episodes} lifetimes in the background...")
        
        # Entraînement instantané avant de démarrer le nœud réel
        self.train_agent()
        
        self.get_logger().info("✅ TRAINING COMPLETE: Optimal Policy Learned via Bellman Equation.")
        
        # Mode Exécution : Le robot va maintenant utiliser ce qu'il a appris
        self.current_state = self.start_state
        self.mission_accomplished = False
        
        # Timer pour visualiser le mouvement à 1 Hz
        self.timer = self.create_timer(1.0, self.execute_optimal_policy)

    def get_reward(self, state):
        """Fonction de récompense (Le système nerveux du robot)"""
        if state == self.goal_state:
            return 100.0  # Jackpot (Objectif atteint)
        elif state in self.obstacles:
            return -100.0 # Douleur (Collision mur)
        else:
            return -1.0   # Fatigue (Chaque pas coûte de l'énergie, pousse à trouver le chemin le plus court)

    def step(self, state, action_idx):
        """Simule le déplacement dans l'environnement"""
        action = self.actions[action_idx]
        next_state = (state[0] + action[0], state[1] + action[1])
        
        # Vérification des limites du labyrinthe
        if next_state[0] < 0 or next_state[0] >= self.grid_size or next_state[1] < 0 or next_state[1] >= self.grid_size:
            return state, -10.0, False # Punition pour avoir essayé de sortir de la carte
            
        reward = self.get_reward(next_state)
        done = next_state == self.goal_state or next_state in self.obstacles
        
        return next_state, reward, done

    def train_agent(self):
        """La boucle d'apprentissage (Markov Decision Process)"""
        for episode in range(self.episodes):
            state = self.start_state
            done = False
            
            while not done:
                # Stratégie Epsilon-Greedy (Exploration vs Exploitation)
                if random.uniform(0, 1) < self.epsilon:
                    action_idx = random.randint(0, 3) # Action aléatoire
                else:
                    action_idx = np.argmax(self.q_table[state[0], state[1]]) # Meilleure action connue
                
                next_state, reward, done = self.step(state, action_idx)
                
                # LA FORMULE DE BELLMAN : Mise à jour de la mémoire
                best_next_action = np.argmax(self.q_table[next_state[0], next_state[1]])
                td_target = reward + self.gamma * self.q_table[next_state[0], next_state[1], best_next_action]
                td_error = td_target - self.q_table[state[0], state[1], action_idx]
                
                self.q_table[state[0], state[1], action_idx] += self.alpha * td_error
                
                state = next_state
            
            # Réduction de l'exploration au fil du temps
            if self.epsilon > self.epsilon_min:
                self.epsilon *= self.epsilon_decay

    def execute_optimal_policy(self):
        """Le robot se déplace de manière autonome en lisant sa Q-Table"""
        if self.mission_accomplished:
            return
            
        if self.current_state == self.goal_state:
            self.get_logger().info("🏆 GOAL REACHED! The AI perfectly navigated the maze without collisions.")
            self.mission_accomplished = True
            self.timer.cancel()
            return
            
        # L'IA ne réfléchit plus, elle sait (Exploitation pure à 100%)
        action_idx = np.argmax(self.q_table[self.current_state[0], self.current_state[1]])
        action = self.actions[action_idx]
        
        # Mouvement
        self.current_state = (self.current_state[0] + action[0], self.current_state[1] + action[1])
        
        # Log et Publication
        self.get_logger().info(f"AI moved to coordinates: X={self.current_state[0]}, Y={self.current_state[1]}")
        
        msg = Point()
        msg.x = float(self.current_state[0])
        msg.y = float(self.current_state[1])
        msg.z = 0.0
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QLearningNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
