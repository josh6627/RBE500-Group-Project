import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import numpy as np

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

    def listener_callback(self):
        pass

    def _compute_forward_kinematics(self, joint_vars: list) -> np.ndarray:
        pass


def main(args=None):
    rclpy.init(args)

    node = ForwardKinematicsNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()