from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("px4ctrl")) / "config" / "px4_native_position_mission.yaml"
    return LaunchDescription([
        Node(
            package="px4ctrl",
            executable="px4_native_position_node",
            name="px4_native_position_node",
            output="screen",
            parameters=[str(config)],
        )
    ])
