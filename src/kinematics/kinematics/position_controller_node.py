import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from scara_params.srv import SetTargetPosition

import csv
from pathlib import Path


class PositionControllerNode(Node):
    """PD position controller for the SCARA robot's prismatic joint."""

    def __init__(self):
        super().__init__("position_controller_node")

        # modifiable constants all in one place. Could be changed in the launch file later.
        self.declare_parameter("joint_name", "joint_3")
        self.declare_parameter("model_name", "arm")
        self.declare_parameter("kp", 50.0)
        self.declare_parameter("kd", 4.0)
        self.declare_parameter("max_effort", 20.0)
        self.declare_parameter("control_period", 0.01)
        self.declare_parameter("gravity_compensation", -4.95)

        self.joint_name = (
            self.get_parameter("joint_name").get_parameter_value().string_value
        )
        self.model_name = (
            self.get_parameter("model_name").get_parameter_value().string_value
        )

        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value
        self.max_effort = (
            self.get_parameter("max_effort").get_parameter_value().double_value
        )
        self.control_period = (
            self.get_parameter("control_period").get_parameter_value().double_value
        )
        self.gravity_compensation = (
            self.get_parameter("gravity_compensation")
            .get_parameter_value()
            .double_value
        )

        # Current joint information.
        self.current_position = 0.0
        self.have_joint_state = False

        # Controller information.
        self.target_position = 0.0
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()
        self.have_previous_error = False

        # Gazebo's ApplyJointForce plugin listens to this topic.
        self.effort_topic = (
            f"/model/{self.model_name}" f"/joint/{self.joint_name}/cmd_force"
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.effort_publisher = self.create_publisher(
            Float64,
            self.effort_topic,
            10,
        )

        # The user calls this service to give the controller a new target.
        self.target_service = self.create_service(
            SetTargetPosition,
            "set_joint_target",
            self.target_callback,
        )

        # Run the PD controller at 100 Hz by default.
        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        # Creating log file
        self.start_time = self.get_clock().now()
        self.log_path = (
            Path.home()
            / "Robotics"
            / "rbe500"
            / "group-project"
            / "joint_position_log.csv"
        )
        self.log_file = open(
            self.log_path,
            "w",
            newline="",
        )
        self.log_writer = csv.writer(self.log_file)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)

        # CSV column headings.
        self.log_writer.writerow(
            [
                "time_s",
                "reference_position_m",
                "current_position_m",
            ]
        )
        self.log_file.flush()
        self.get_logger().info(f"Recording joint data to: {self.log_path}")

        self.get_logger().info(f"Controlling joint '{self.joint_name}'")
        self.get_logger().info(f"Publishing force to '{self.effort_topic}'")
        self.get_logger().info(
            f"PD gains: kp={self.kp}, kd={self.kd}, " f"max_effort={self.max_effort}"
        )

    def joint_state_callback(self, msg: JointState):
        """Read joint_3's position and velocity by joint name."""

        if self.joint_name not in msg.name:
            return

        joint_index = msg.name.index(self.joint_name)

        if joint_index >= len(msg.position):
            self.get_logger().warning(
                f"JointState contains '{self.joint_name}' in msg.name, "
                "but it has no corresponding position."
            )
            return

        self.current_position = float(msg.position[joint_index])

        self.have_joint_state = True

    def target_callback(self, request, response):
        """Receive a new desired joint position."""

        requested_target = float(request.target)

        # reject target if it is not within range
        if requested_target < 0.0 or requested_target > 0.5:
            self.get_logger().warning(
                f"Rejected target {requested_target:.3f}. "
                "Valid range is 0.0 to 0.5 m."
            )
            response.success = False
            return response

        if not math.isfinite(requested_target):
            self.get_logger().warning(f"Rejected invalid target: {requested_target}")
            response.success = False
            return response

        self.target_position = requested_target

        # Reset the numerical derivative whenever the target changes.
        self.have_previous_error = False
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

        self.get_logger().info(f"New target position: {self.target_position:.4f} m")

        response.success = True
        return response

    def control_loop(self):
        """Calculate and publish the PD effort command."""

        if not self.have_joint_state:
            return

        position_error = self.target_position - self.current_position

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.previous_time).nanoseconds / 1.0e9

        # make sure prev error exists before running calculation
        if self.have_previous_error and elapsed_time > 0.0:
            derivative_error = (position_error - self.previous_error) / elapsed_time

        else:
            derivative_error = 0.0

        effort = (
            self.kp * position_error
            + self.kd * derivative_error
            + self.gravity_compensation  # small gravity compensation when error is zero
        )

        # Prevent an excessively large force command.
        effort = max(
            -self.max_effort,
            min(self.max_effort, effort),
        )

        effort_message = Float64()
        effort_message.data = float(effort)
        self.effort_publisher.publish(effort_message)

        # Record reference and measured positionsThe VEX extension generates that file with the correct VEX include directories. Do not replace it with a normal Linux GCC configuration.

        elapsed_time_from_start = (current_time - self.start_time).nanoseconds / 1.0e9
        self.log_writer.writerow(
            [
                f"{elapsed_time_from_start:.6f}",
                f"{self.target_position:.6f}",
                f"{self.current_position:.6f}",
            ]
        )
        self.log_file.flush()

        self.previous_error = position_error
        self.previous_time = current_time
        self.have_previous_error = True

    def close_log_file(self):
        """Flush and close the controller data file."""

        if hasattr(self, "log_file") and not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()

            self.get_logger().info(f"Saved joint data to: {self.log_path}")


def main(args=None):
    rclpy.init(args=args)

    node = PositionControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_log_file()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
