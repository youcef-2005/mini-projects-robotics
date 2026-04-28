import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class GridMapper(Node):
    def __init__(self):
        super().__init__('grid_mapper')
        
        # Publisher pour la carte finale
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Paramètres de la carte (10m x 10m, résolution 0.1m)
        self.res = 0.1
        self.width = 100 
        self.height = 100
        self.origin_x = -5.0
        self.origin_y = -5.0
        
        # Log-Odds Mapping (Probabilités : 0 = libre, 100 = occupé, -1 = inconnu)
        self.prob_map = np.zeros((self.width, self.height), dtype=np.float32)
        
        # Simulation d'un LiDAR interne pour garantir que le code tourne seul
        self.timer = self.create_timer(0.5, self.update_map_simulation)
        
        self.get_logger().info("Mapping Engine Started: Initializing 2D Occupancy Grid...")

    def bresenham(self, x0, y0, x1, y1):
        """Algorithme de tracé de ligne pour marquer les zones libres"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        return cells

    def update_map_simulation(self):
        """Simule un balayage laser à 360° et met à jour la carte"""
        # Position du robot (centre)
        robot_x_idx = self.width // 2
        robot_y_idx = self.height // 2
        
        for angle in range(0, 360, 5): # Balayage tous les 5 degrés
            rad = math.radians(angle)
            
            # Simulation d'un obstacle à 3 mètres
            dist = 3.0 
            
            # Coordonnées de l'impact laser
            end_x = robot_x_idx + int((dist * math.cos(rad)) / self.res)
            end_y = robot_y_idx + int((dist * math.sin(rad)) / self.res)
            
            # 1. Marquer les cellules traversées comme "libres" (Probabilité diminue)
            free_cells = self.bresenham(robot_x_idx, robot_y_idx, end_x, end_y)
            for cx, cy in free_cells:
                if 0 <= cx < self.width and 0 <= cy < self.height:
                    self.prob_map[cx, cy] -= 0.2 # Log-odds decrease
            
            # 2. Marquer la cellule d'impact comme "occupée" (Probabilité augmente)
            if 0 <= end_x < self.width and 0 <= end_y < self.height:
                self.prob_map[end_x, end_y] += 1.0 # Log-odds increase

        self.publish_map()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.res
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        
        # Conversion des log-odds en valeurs ROS [0, 100]
        clamped_map = np.clip(self.prob_map, -1, 1) # Normalisation simple
        data = (clamped_map.flatten() * 100).astype(np.int8)
        
        # Remplacer les zones non explorées par -1
        data[data < 0] = 0
        msg.data = data.tolist()
        
        self.map_pub.publish(msg)

def main():
    rclpy.init()
    node = GridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
