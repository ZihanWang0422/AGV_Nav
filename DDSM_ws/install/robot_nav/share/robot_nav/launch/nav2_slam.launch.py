#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_nav_dir = get_package_share_directory('robot_nav')
    
    # 声明参数
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # 声明启动参数
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
        
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM')
        
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robot_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
        
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # 包含Nav2 Bringup launch文件
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart}.items())

    # 添加odom_to_tf节点
    odom_to_tf_node = Node(
        package='robot_nav',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen'
    )

    # 添加map_converter节点
    map_converter_node = Node(
        package='robot_nav',
        executable='map_converter',
        name='map_converter',
        output='screen'
    )

    # 添加cmd_vel_mux节点
    cmd_vel_mux_node = Node(
        package='robot_nav',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen'
    )

    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加声明的启动参数
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # 添加导航启动文件
    ld.add_action(nav2_bringup_launch)
    # 添加odom_to_tf节点
    ld.add_action(odom_to_tf_node)
    # 添加map_converter节点
    ld.add_action(map_converter_node)
    # 添加cmd_vel_mux节点
    ld.add_action(cmd_vel_mux_node)
    
    return ld 