#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            'record_data', default_value='true',
            description='Start the ROS numeric flight-data ULog recorder'),
        DeclareLaunchArgument(
            'log_directory', default_value='',
            description='ULog directory; empty uses ~/.ros/flight_logs'),
        Node(
            package='px4ctrl',
            executable='px4ctrl_node',
            name='px4ctrl_node',
            parameters=[config],  # 加载参数文件
            output='screen'
        ),
        Node(
            package='px4ctrl',
            executable='px4ctrlrate_node',
            name='px4ctrlrate_node',
            output='screen'
        ),
        Node(
            package='flight_data_recorder',
            executable='flight_data_recorder_node',
            name='flight_data_recorder',
            parameters=[os.path.join(
                get_package_share_directory('flight_data_recorder'), 'config', 'recorder.yaml'), {
                'output_directory': LaunchConfiguration('log_directory'),
            }],
            condition=IfCondition(LaunchConfiguration('record_data')),
            output='screen'
        ),
   ])
