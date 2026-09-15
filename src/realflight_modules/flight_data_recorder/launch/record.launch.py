from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('flight_data_recorder')) / 'config' / 'recorder.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('log_directory', default_value=''),
        DeclareLaunchArgument('status_topic', default_value='/fmu/out/vehicle_status_v1'),
        Node(package='flight_data_recorder', executable='flight_data_recorder_node',
             name='flight_data_recorder', output='screen', parameters=[str(config), {
                 'output_directory': LaunchConfiguration('log_directory'),
                 'status_topic': LaunchConfiguration('status_topic'),
             }]),
    ])
