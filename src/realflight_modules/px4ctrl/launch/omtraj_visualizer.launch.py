from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = Path(get_package_share_directory('px4ctrl')) / 'config'
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz2')
    exit_arg = DeclareLaunchArgument(
        'exit_after_solve', default_value='false',
        description='Exit the optimizer after generating the trajectory')

    planner = Node(
        package='px4ctrl', executable='omtraj_visualizer_node',
        name='omtraj_visualizer', output='screen',
        parameters=[str(config_dir / 'omtraj.yaml'),
                    {'exit_after_solve': ParameterValue(
                        LaunchConfiguration('exit_after_solve'), value_type=bool)}])
    rviz = Node(
        package='rviz2', executable='rviz2', name='omtraj_rviz',
        arguments=['-d', str(config_dir / 'omtraj.rviz')], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([rviz_arg, exit_arg, planner, rviz])
