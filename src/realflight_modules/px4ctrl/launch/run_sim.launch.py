"""One entry point: GUI simulator and both control loops share /clock."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    def include(package, filename, **arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory(package)) / 'launch' / filename)),
            launch_arguments=arguments.items())
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(
            Path(get_package_share_directory('px4ctrl')) / 'config' / 'params.yaml')),
        include('px4ctrl', 'run_ctrl.launch.py', use_sim_time='true',
                params_file=LaunchConfiguration('params_file')),
        include('quadsim_mujoco', 'mujoco.launch.py'),
    ])
