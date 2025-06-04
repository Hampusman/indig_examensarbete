import rclpy
from rclpy.node import Node

class MainNode(Node):
    def __init__(self):
        super().__init__("main_node")

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()