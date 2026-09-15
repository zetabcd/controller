from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('flight_data_recorder')) / 'config' / 'recorder.yaml'

    def recorder(context):
        overrides = {'status_topic': LaunchConfiguration('status_topic').perform(context)}
        directory = LaunchConfiguration('log_directory').perform(context)
        if directory:
            overrides['output_directory'] = directory
        return [Node(
            package='flight_data_recorder', executable='flight_data_recorder_node',
            name='flight_data_recorder', output='screen', parameters=[str(config), overrides])]

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_directory', default_value='',
            description='Project-relative or absolute path; empty keeps recorder.yaml'),
        DeclareLaunchArgument('status_topic', default_value='/fmu/out/vehicle_status_v1'),
        OpaqueFunction(function=recorder),
    ])
