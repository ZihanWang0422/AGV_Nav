#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 包含Nav2 Bringup launch文件
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        }.items())

    # 创建启动描述
    ld = LaunchDescription()
    ld.add_action(nav2_bringup_launch)
    
    return ld 