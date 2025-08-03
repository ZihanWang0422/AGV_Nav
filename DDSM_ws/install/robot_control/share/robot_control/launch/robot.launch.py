from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            namespace='robot_control',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
            emulate_tty=True
        )
    ]) 