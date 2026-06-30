import rclpy
from rclpy.node import Node


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')


def main(args=None):
    rclpy.init(args)

    node = RobotNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()