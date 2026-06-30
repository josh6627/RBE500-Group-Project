import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import numpy as np

import math
from .custom_utils import get_transformation_matrix, get_rotation_matrix


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__("forward_kinematics_node")

    # helper function that calculates the HT matrix based on the DH params
    def dh(self, theta, d, a, alpha):
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1],
            ]
        )

    def listener_callback(self):
        pass

    def _compute_forward_kinematics(self, joint_vars: list) -> np.ndarray:

        # find the individual transformations of each matrix
        T0_1 = self.dh(joint_vars[0], 475, 150, 0)
        T1_2 = self.dh(joint_vars[1], 0, 600, 0)
        T2_3 = self.dh(0, -joint_vars[2], 0, math.pi)

        return T0_1 @ T1_2 @ T2_3


def main(args=None):
    rclpy.init(args)

    node = ForwardKinematicsNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
