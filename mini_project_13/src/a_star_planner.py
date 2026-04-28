import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import math

class NodeAStar:
    """Structure de données pour un nœud de la grille"""
    def __init__(self, x, y, cost=0.0, heuristic=0.0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost          # g(n)
        self.heuristic = heuristic # h(n)
        self.parent = parent
        
    @property
    def total(self):              # f(n) = g(n) + h(n)
        return self.cost + self.heuristic

    def __lt__(self, other):
        return self.total < other.total

class AutonomousPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        
        # Publishers pour la visualisation sous RViz
        self.map_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Dimensions de la carte (20x20 mètres)
        self.grid_size_x = 20
        self.grid_size_y = 20
        self.resolution = 1.0 # 1 mètre par cellule
        
        # Création d'une carte virtuelle avec des obstacles
        self.grid = [[0 for _ in range(self.grid_size_y)] for _ in range(self.grid_size_x)]
        self.generate_obstacles()
        
        # Points de départ et d'arrivée
        self.start = (2, 2)
        self.goal = (18, 18)
        
        # Timer pour publier la solution en boucle
        self.timer = self.create_timer(2.0, self.publish_planning)
        self.get_logger().info("A* Path Planner Initialized. Calculating optimal route...")

    def generate_obstacles(self):
        """Place des murs (100) dans la grille"""
        for i in range(5, 15):
            self.grid[i][10] = 100 # Mur vertical
        for i in range(5, 15):
            self.grid[15][i] = 100 # Mur horizontal

    def heuristic(self, a, b):
        """Distance de Manhattan"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy
            # Vérification des limites et des obstacles
            if 0 <= nx < self.grid_size_x and 0 <= ny < self.grid_size_y:
                if self.grid[nx][ny] == 0:
                    # Coût de déplacement : 1 pour droit, sqrt(2) pour diagonal
                    move_cost = 1.0 if dx == 0 or dy == 0 else 1.414
                    neighbors.append((nx, ny, move_cost))
        return neighbors

    def a_star_search(self):
        start_node = NodeAStar(self.start[0], self.start[1])
        goal_node = NodeAStar(self.goal[0], self.goal[1])
        
        open_list = []
        heapq.heappush(open_list, start_node)
        closed_set = set()
        
        while open_list:
            current_node = heapq.heappop(open_list)
            
            if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
                # Chemin trouvé, on remonte les parents
                path = []
                while current_node is not None:
                    path.append((current_node.x, current_node.y))
                    current_node = current_node.parent
                return path[::-1]
            
            closed_set.add((current_node.x, current_node.y))
            
            for nx, ny, move_cost in self.get_neighbors(current_node):
                if (nx, ny) in closed_set:
                    continue
                    
                neighbor = NodeAStar(nx, ny, current_node.cost + move_cost, self.heuristic((nx, ny), self.goal), current_node)
                
                # Vérifie si un meilleur chemin existe déjà
                if any(n.x == nx and n.y == ny and n.total <= neighbor.total for n in open_list):
                    continue
                    
                heapq.heappush(open_list, neighbor)
                
        return None # Aucun chemin possible

    def publish_planning(self):
        now = self.get_clock().now().to_msg()
        
        # 1. Publication de la carte (OccupancyGrid)
        map_msg = OccupancyGrid()
        map_msg.header.stamp = now
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.grid_size_x
        map_msg.info.height = self.grid_size_y
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        
        # Aplatir la grille 2D en liste 1D pour ROS
        flat_grid = []
        for y in range(self.grid_size_y):
            for x in range(self.grid_size_x):
                flat_grid.append(self.grid[x][y])
        map_msg.data = flat_grid
        self.map_pub.publish(map_msg)
        
        # 2. Calcul et Publication du chemin (Path)
        waypoints = self.a_star_search()
        
        if waypoints:
            path_msg = Path()
            path_msg.header.stamp = now
            path_msg.header.frame_id = "map"
            
            for (x, y) in waypoints:
                pose = PoseStamped()
                pose.header.stamp = now
                pose.header.frame_id = "map"
                pose.pose.position.x = float(x) * self.resolution
                pose.pose.position.y = float(y) * self.resolution
                path_msg.poses.append(pose)
                
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Path published successfully. Total waypoints: {len(waypoints)}")
        else:
            self.get_logger().warn("Goal is unreachable!")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
