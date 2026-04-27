import rclpy
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from std_msgs.msg import String

class IndustrialSensorNode(Node):
    def __init__(self):
        super().__init__('industrial_sensor')
        self._pub = None
        self._timer = None
        self.get_logger().info("Node instantiated: [UNCONFIGURED] state.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Transitioning to [INACTIVE]: Allocating memory and setting up hardware drivers...")
        self._pub = self.create_lifecycle_publisher(String, 'sensor_data', 10)
        self._timer = self.create_timer(1.0, self.timer_callback)
        self._timer.cancel()  # Timer is created but paused until Activation
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Transitioning to [ACTIVE]: Turning on sensor lasers and enabling publisher...")
        self._pub.on_activate()
        self._timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Transitioning to [INACTIVE]: Safely stopping sensor and pausing publisher...")
        self._pub.on_deactivate()
        self._timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Transitioning to [UNCONFIGURED]: Freeing hardware memory and destroying entities...")
        self.destroy_publisher(self._pub)
        self.destroy_timer(self._timer)
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String(data="Sensor Reading: 100% Nominal")
        self._pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = IndustrialSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
