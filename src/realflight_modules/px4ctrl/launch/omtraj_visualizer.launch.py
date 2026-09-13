from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = Path(get_package_share_directory('px4ctrl')) / 'config'
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz2')

    planner = Node(
        package='px4ctrl', executable='omtraj_visualizer_node',
        name='omtraj_visualizer', output='screen',
        parameters=[str(config_dir / 'omtraj.yaml')])
    rviz = Node(
        package='rviz2', executable='rviz2', name='omtraj_rviz',
        arguments=['-d', str(config_dir / 'omtraj.rviz')], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([rviz_arg, planner, rviz])
