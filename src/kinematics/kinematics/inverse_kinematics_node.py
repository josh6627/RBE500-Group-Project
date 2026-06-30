import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import numpy as np


class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__("inverse_kinematics_node")

    def listener_callback(self):
        pass

    def _compute_inverse_kinematics(self, joint_vars: list) -> np.ndarray:
        pass


def main(args=None):
    rclpy.init(args)

    node = InverseKinematicsNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
