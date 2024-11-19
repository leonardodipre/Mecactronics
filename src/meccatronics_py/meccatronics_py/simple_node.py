# simple_node.py
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Nodo ROS2 semplice avviato!')
        self.timer = self.create_timer(1.0, self.timer_callback)  # Chiama ogni 1 secondo

    def timer_callback(self):
        self.get_logger().info('Hello from Simple Node!')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
