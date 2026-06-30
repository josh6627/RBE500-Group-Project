import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np

import math
from .custom_utils.transforms import get_translation_matrix, get_rotation_matrix


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__("forward_kinematics_node")

        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.listener_callback, 10
        )
        self.pose_pub = self.create_publisher(PoseStamped, "/ee_pose", 10)

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

    def listener_callback(self, msg: JointState):
        joint_angles = msg.position
        ht_matrix = self._compute_forward_kinematics(joint_angles)
        pose_msg = self.ht_matrix_to_pose_msg(ht_matrix)
        self.pose_pub.publish(pose_msg)

    def _compute_forward_kinematics(self, joint_vars: list) -> np.ndarray:

        # find the individual transformations of each matrix
        T0_1 = self.dh(joint_vars[0], 100, 100, 0)
        T1_2 = self.dh(joint_vars[1], 0, 100, 0)
        T2_3 = self.dh(0, -joint_vars[2], 0, math.pi)

        return T0_1 @ T1_2 @ T2_3

    def ht_matrix_to_pose_msg(self, ht_matrix: np.ndarray) -> PoseStamped:
        pose_msg = PoseStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"

        pose_msg.pose.position.x = float(ht_matrix[0, 3])
        pose_msg.pose.position.y = float(ht_matrix[1, 3])
        pose_msg.pose.position.z = float(ht_matrix[2, 3])

        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(ht_matrix[0:3, 0:3])

        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        return pose_msg

    # convert the rotation matrix into a quaturnion
    def rotation_matrix_to_quaternion(self, R):
        trace = R[0, 0] + R[1, 1] + R[2, 2]

        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s

        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s

        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s

        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        return float(qx), float(qy), float(qz), float(qw)


def main(args=None):
    rclpy.init(args=args)

    node = ForwardKinematicsNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
