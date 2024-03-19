from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory("ros2_drone_sim")
    sdf_path = os.path.join(package_share_dir, "worlds", "iris_runway.sdf")

    return LaunchDescription([
           ExecuteProcess(
                cmd=["gz","sim","-v4","-r",sdf_path,],
                output="screen",
            ),

    ])
