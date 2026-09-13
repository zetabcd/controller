#!/usr/bin/python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("px4ctrl"))
    rviz_config = package_share / "config" / "realflight_trajectory.rviz"

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Start RViz2 on this computer"
    )
    debug_topic_arg = DeclareLaunchArgument(
        "debug_topic", default_value="/debugPx4/ctrl"
    )
    position_topic_arg = DeclareLaunchArgument(
        "position_topic", default_value="/fmu/out/vehicle_local_position"
    )

    visualizer = Node(
        package="px4ctrl",
        executable="realflight_trajectory_visualizer_node",
        name="realflight_trajectory_visualizer",
        output="screen",
        parameters=[{
            "debug_topic": LaunchConfiguration("debug_topic"),
            "position_topic": LaunchConfiguration("position_topic"),
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="realflight_trajectory_rviz",
        arguments=["-d", str(rviz_config)],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription([
        rviz_arg,
        debug_topic_arg,
        position_topic_arg,
        visualizer,
        rviz,
    ])
