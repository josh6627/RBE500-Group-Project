import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to ros_gz_sim instead of gazebo_ros
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    urdf_path = '/home/nsmit/RBE500-Group-Project/src/scara_params/urdf/scara.urdf'

    return LaunchDescription([
        # 1. Launch Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': 'empty.sdf'}.items(),
        ),
        # 2. Spawn the entity using the create node
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-name', 'arm',
                 '-file', urdf_path],
            output='screen'
        )
    ])