import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class ParallelNode(Node):
    def __init__(self):
        super().__init__('parallel_node')
        # Groupe réentrant = callbacks en parallèle
        group = ReentrantCallbackGroup()
        
        self.create_timer(
            0.05, self.fast_cb,      # 20 Hz
            callback_group=group)
        self.create_timer(
            0.5, self.slow_cb,       # 2 Hz
            callback_group=group)

    def fast_cb(self):
        self.get_logger().info('Fast tick')

    def slow_cb(self):
        time.sleep(0.4) # Travail lourd
        self.get_logger().info('Slow done')

def main():
    rclpy.init()
    node = ParallelNode()
    # 4 threads pour parallélisme
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
