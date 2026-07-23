import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    scara_pkg = get_package_share_directory("scara_params")
    ros_gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    urdf_file = os.path.join(scara_pkg, "urdf", "scara.urdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v 4 empty.sdf"}.items(),
    )

    spawn_arm = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "empty",
            "-file",
            urdf_file,
            "-name",
            "arm",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Gazebo -> ROS
            "/joint_states" "@sensor_msgs/msg/JointState" "[gz.msgs.Model",
            # ROS -> Gazebo
            "/model/arm/joint/joint_3/cmd_force"
            "@std_msgs/msg/Float64"
            "]gz.msgs.Double",
        ],
        output="screen",
    )

    fk_node = Node(
        package="kinematics_cpp",
        executable="fwd_kinematics",
        name="fwd_kinematics",
        output="screen",
    )

    ik_node = Node(
        package="kinematics_cpp",
        executable="inv_kinematics",
        name="fwd_kinematics",
        output="screen",
    )

    controller_node = Node(
        package="kinematics",
        executable="controller",
        output="screen",
    )
    return LaunchDescription(
        [
            # gazebo,
            # TimerAction(period=6.0, actions=[spawn_arm]),
            # TimerAction(period=8.0, actions=[joint_state_bridge]),
            TimerAction(period=9.0, actions=[fk_node, ik_node]),
        ]
    )
