from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='串口设备路径'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='串口波特率'
    )

    # 创建串口控制节点
    serial_controller_node = Node(
        package='robot_control',
        executable='serial_controller',
        name='serial_controller',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }],
        output='screen',
        prefix='python3'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        serial_controller_node
    ]) 