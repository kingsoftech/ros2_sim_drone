import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory("ros2_drone_sim")
    sdf_path = os.path.join(package_share_dir, "worlds","iris_runway.sdf")
    sdf_path = open(sdf_path).read()


    return LaunchDescription(
        [
            
             Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gzserver',
            output='screen',
            arguments=[sdf_path]  # Pass the path to your SDF file
        )
        ]
    )