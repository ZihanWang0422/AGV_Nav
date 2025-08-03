# main_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 包含 lsn10_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/lsn10_launch.py']  # 确保路径正确
            )
        ),
        # 启动 controller_manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',  # 注意：ROS 2 中是 ros2_control_node，而不是 controller_manager
            name='controller_manager',
            output='screen',
            parameters=[{'use_sim_time': False}],  # 如果使用仿真时间，设置为 True
        ),
    ])
