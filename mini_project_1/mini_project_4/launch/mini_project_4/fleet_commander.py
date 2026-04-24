import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class RobotState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    CHARGING = "charging"

class FleetCommander(Node):
    def __init__(self):
        super().__init__('fleet_commander')
        self.robots = ['robot1', 'robot2', 'robot3']
        self.status_pub = self.create_publisher(String, '/fleet/status', 10)
        self.timer = self.create_timer(2.0, self.publish_status)
        self.get_logger().info("Fleet Commander Initialized and Monitoring...")

    def publish_status(self):
        # Simulation du monitoring de la flotte
        msg = String()
        msg.data = f"Fleet Status: All robots operational. Active: {len(self.robots)}"
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    commander = FleetCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()
