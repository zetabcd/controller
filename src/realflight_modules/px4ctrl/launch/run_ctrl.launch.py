#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# sun: 同时启动顶层外环和底层角速度环；只有顶层节点直接加载 YAML，
# sun: 底层节点启动后通过参数服务读取同一份飞行器和控制参数。

def generate_launch_description():
    # 获取参数文件的路径
    config = os.path.join(
        get_package_share_directory('px4ctrl'),  # 替换为您的包名
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=config,
            description='Controller YAML; restart nodes after changing parameters'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulator /clock; keep false for real flight'),
        DeclareLaunchArgument(
            'record_data', default_value='true',
            description='Start the ROS numeric flight-data ULog recorder'),
        DeclareLaunchArgument(
            'log_directory', default_value='',
            description='Project-relative or absolute ULog directory; empty keeps recorder.yaml'),
        Node(
            package='px4ctrl',
            executable='px4ctrl_node',
            name='px4ctrl_node',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        Node(
            package='px4ctrl',
            executable='px4ctrlrate_node',
            name='px4ctrlrate_node',
            parameters=[os.path.join(
                get_package_share_directory('px4ctrl'), 'config', 'ratectrl_diagnostics.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('flight_data_recorder'), 'launch', 'record.launch.py')),
            launch_arguments={'log_directory': LaunchConfiguration('log_directory')}.items(),
            condition=IfCondition(LaunchConfiguration('record_data')),
        ),
   ])
