import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
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

    controller_params = os.path.join(
        get_package_share_directory("kinematics_cpp"),
        "config",
        "position_controller_params.yaml",
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

    robot_geometry = {
        "link_1_length": 1.0,
        "link_2_length": 1.0,
        "link_3_length": 1.0,
        "joint_3_offset": 0.04,
        "joint_3_min": 0.0,
        "joint_3_max": 0.5,
    }
    
    fk_node = Node(
        package="kinematics_cpp",
        executable="fwd_kinematics",
        name="fwd_kinematics",
        parameters=[robot_geometry],
        output="screen",
    )

    ik_node = Node(
        package="kinematics_cpp",
        executable="inv_kinematics",
        name="fwd_kinematics",
        parameters=[robot_geometry],
        output="screen",
    )

    controller_node = Node(
        package="kinematics_cpp",
        executable="controller",
        parameters=[controller_params],
        output="screen",
    )
    velocity_kinematics_node = Node(
        package="kinematics_cpp",
        executable="vel_kinematics",
        parameters=[robot_geometry],
        output="screen",

    )
    return LaunchDescription(
        [
        gazebo,

    RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[spawn_arm],
        )
    ),

    RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm,
            on_exit=[joint_state_bridge],
        )
    ),

    RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_bridge,
            on_start=[
                fk_node,
                ik_node,
                controller_node,
                velocity_kinematics_node,
            ],
        )
    ),
        ]
    )

"""
return LaunchDescription([
    gazebo,
    joint_state_bridge,
    fk_node,
    ik_node,
    controller_node,

    TimerAction(
        period=2.0,
        actions=[spawn_arm],
    ),
])
"""