"""Same controller/planner pipeline for configured gates and motion capture."""
from pathlib import Path
import yaml
import time
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    simulation = value('simulation').lower() == 'true'
    controller_file, gap_file = value('controller_params'), value('gap_params')
    with open(controller_file, encoding='utf-8') as stream:
        control = yaml.safe_load(stream)['px4ctrl_node']['ros__parameters']
    with open(gap_file, encoding='utf-8') as stream:
        gates = yaml.safe_load(stream)['gap_planner']['ros__parameters']
    mass = control['uav']['mass']
    controller_kind = value('controller_kind')
    drag_enabled = control[controller_kind].get('drag_compensation', True)
    model = {
        'vehicle.mass': mass, 'vehicle.arm': control['uav']['l'],
        'vehicle.prop_radius': control['uav']['rp'], 'vehicle.gravity': control['gra'],
        'vehicle.drag_acceleration': [control['aero'][key] / mass if drag_enabled else 0.0
                                      for key in ('kdx', 'kdy', 'kdz')],
        'vehicle.lift_acceleration': control['aero']['kh'] / mass if drag_enabled else 0.0,
        'simulation': simulation, 'use_sim_time': simulation,
    }
    motor, uav = control['motor'], control['uav']
    ct = motor['Ct_c'] * 4 * control['aero']['rho'] * uav['rp']**4 / math.pi**2
    speed_max = sum(motor[key] for key in ('rc2speed_a', 'rc2speed_b', 'rc2speed_c'))
    limits = control['trajectory']['limits']
    model.update({
        'vehicle.inertia': [uav[key] for key in ('Jvx', 'Jvy', 'Jvz')],
        'vehicle.arm_angle': math.radians(uav['beta_deg']),
        'vehicle.torque_to_thrust': 2*uav['rp']*motor['Cq_c']/motor['Ct_c'],
        'vehicle.motor_min': ct*motor['rc2speed_c']**2,
        'vehicle.motor_max': ct*speed_max**2*limits.get('motor_fraction', 0.7),
        'vehicle.execution_rate_max': control[controller_kind].get('body_rate_max', [14.0]*3),
        'vehicle.angular_acceleration_max': limits.get('angular_acceleration', 100.0),
    })
    share = Path(get_package_share_directory('px4ctrl'))
    actions = [
        Node(package='px4ctrl', executable='px4ctrl_node', name='px4ctrl_node', output='screen',
             parameters=[controller_file, {'trajectory.type': 'external', 'use_sim_time': simulation,
                'trajectory.external.simulation': simulation,
                'trajectory.external.takeoff_height': gates['mission']['flight_height']}]),
        Node(package='px4ctrl', executable='px4ctrlrate_node', name='px4ctrlrate_node', output='screen',
             parameters=[str(share / 'config' / 'ratectrl_diagnostics.yaml'), {'use_sim_time': simulation}]),
        Node(package='gap_planner', executable='gap_planner_node', name='gap_planner', output='screen',
             parameters=[gap_file, model]),
        Node(package='gap_planner', executable='gap_trajectory_sender', name='gap_trajectory_sender',
             output='screen', parameters=[gap_file, {'use_sim_time': simulation}]),
    ]
    if simulation:
        sim_share = Path(get_package_share_directory('quadsim_mujoco'))
        actions.append(Node(package='quadsim_mujoco', executable='quadsim_node', name='quadsim_node',
            output='screen', parameters=[str(sim_share / 'config' / 'params.yaml'),
                {'initial_position': [-1.65, 0.0, 0.05], 'gap_scene_file': gap_file,
                 'log_directory': 'datalog/gap_sim/' + time.strftime('%Y%m%d_%H%M%S'),
                 'headless': value('headless').lower() == 'true'}]))
    if value('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', str(Path(get_package_share_directory('gap_planner')) / 'config' / 'gaps.rviz')],
            parameters=[{'use_sim_time': simulation}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('controller_kind', default_value='mpc',
            description='mpc or nmpc; must match compiled PX4CTRL_PRIMARY_CONTROLLER'),
        DeclareLaunchArgument('controller_params', default_value=str(
            Path(get_package_share_directory('px4ctrl')) / 'config' / 'params.yaml')),
        DeclareLaunchArgument('gap_params', default_value=str(
            Path(get_package_share_directory('gap_planner')) / 'config' / 'gaps.yaml')),
        OpaqueFunction(function=nodes),
    ])
