import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import numpy as np


class InverseKinematicsNode(Node):
    L1 = 1.0
    L2 = 1.0
    L3 = 1.0

    def __init__(self):
        super().__init__('inverse_kinematics_node')
        # Subscribe to the 'desired_pose' topic, expecting messages of type Float32MultiArray
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'desired_pose',
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, msg):
        P = np.array(msg.data)
        if len(P) != 3:
            self.get_logger().error(f'Expected quaternion of length 4, but got {len(quat)}. Received data: {[float(x) for x in quat]}')
            return

        # Data received as [Ex, Ey, Ez], desired position
        # Orientation will always be the same (down)
        self.get_logger().info(f'Desired Position: (x={P[0]}, y={P[1]}, z={P[2]})')

        joint_variables = self._compute_inverse_kinematics(P)

        self.get_logger().info(f'Computed Joint Variables:')

        vars = [[float(x) for x in joint_variables[i]] for i in range(len(joint_variables))]

        infeasible = False
        z = False
        for result in vars:
            if np.isnan(result).any():
                infeasible = True
                break
            elif result[-1] > 0.5:
                infeasible = True
                z = True
        
        if infeasible:
            self.get_logger().error(f'Infeasible Endpoint. {("Z out of range. " if z else "")}Computed values: {vars}')
            return

        self.get_logger().info(str(f'{vars}\n'))

    def _compute_inverse_kinematics(self, P) -> np.ndarray:
        Ex, Ey, Ez = P
        
        # Compute d3
        d3 = self.L1 - Ez

        R_sqr = Ex**2 + Ey**2
        
        #Compute theta1
        D1 = (R_sqr + self.L2**2 - self.L3) / (2*self.L2*np.sqrt(R_sqr))
        C1 = np.array([-1, 1]) * np.sqrt(1 - D1**2)         # +- sqrt(1 - D1**2)
        theta1 = np.arctan2(C1, D1)
        
        #Compute theta2
        D2 = (R_sqr - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        C2 = np.array([-1, 1]) * np.sqrt(1 - D2**2)         # +- sqrt(1 - D2**2)
        theta2 = np.arctan2(C2, D2)

        return [np.array([theta1[0], theta2[0], d3]),
                np.array([theta1[0], theta2[1], d3]),
                np.array([theta1[1], theta2[0], d3]), 
                np.array([theta1[1], theta2[1], d3]),
                ]


def main(args=None):
    rclpy.init(args=args)

    node = InverseKinematicsNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
