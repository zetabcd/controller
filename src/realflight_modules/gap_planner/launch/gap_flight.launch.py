"""穿缝统一启动：只修改下面的配置区，不需要命令行 launch 参数。"""
from pathlib import Path
import yaml
import time
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ==================== 用户配置区 ====================
# 与 input.h 的 PX4CTRL_SIMULATION 一致；这里控制启动行为，不改变已编译的代码。
SIMULATION = True             # True: MuJoCo + YAML 框位姿；False: 实机 + 框刚体动捕
HEADLESS = False              # True: 不弹出 MuJoCo 窗口（仍运行仿真）
RVIZ = True                   # 是否启动 RViz
RECORD_DATA = True            # 启动数据记录；目录和记录触发方式使用 flight_data_recorder/config/recorder.yaml
CONTROLLER_KIND = 'mpc'        # px4ctrlfsm.h: 1=OMMPC 用 mpc；2=acados 用 nmpc

# 文件名相对于对应包的 config 目录；也可以直接填写绝对路径。
CONTROLLER_PARAMS = 'params.yaml'           # 可改成 params_ommpc_flip.yaml
GAP_PARAMS = 'gaps.yaml'                    # 穿缝任务、窗框几何和动捕配置
RATECTRL_PARAMS = 'ratectrl_diagnostics.yaml'
SIM_PARAMS = 'params.yaml'                  # quadsim_mujoco/config 下的仿真参数
RVIZ_CONFIG = 'gaps.rviz'
SIM_INITIAL_POSITION = [-1.65, 0.0, 0.05]   # 世界系 NWU，米；仿真初始地面位置
SIM_LOG_ROOT = 'datalog/gap_sim'             # 每次启动创建时间命名的子目录
# ===================================================


def config_file(package, filename):
    return str(Path(get_package_share_directory(package)) / 'config' / filename)


def generate_launch_description():
    simulation = SIMULATION
    controller_file = config_file('px4ctrl', CONTROLLER_PARAMS)
    gap_file = config_file('gap_planner', GAP_PARAMS)
    with open(controller_file, encoding='utf-8') as stream:
        control = yaml.safe_load(stream)['px4ctrl_node']['ros__parameters']
    with open(gap_file, encoding='utf-8') as stream:
        gates = yaml.safe_load(stream)['gap_planner']['ros__parameters']
    mass = control['uav']['mass']
    controller_kind = CONTROLLER_KIND
    if controller_kind not in ('mpc', 'nmpc'):
        raise ValueError('CONTROLLER_KIND must be mpc (OMMPC) or nmpc (acados)')
    drag_enabled = control[controller_kind].get('drag_compensation', True)
    # 以下 model 传给 gap_planner_node，用于机体碰撞和动力学可执行性检查。
    # 它不是 MuJoCo 场景模型；实物规划也需要这些参数。
    # 从所选控制器 YAML 导出，覆盖 gaps.yaml 中的同名字段。
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
    # 明确列出启动时的覆盖项：后面的字典优先于前面的 YAML。
    # 穿缝入口强制 external；普通 run_ctrl 仍使用 YAML 的 trajectory.type。
    controller_overrides = {
        'trajectory.type': 'external',
        'use_sim_time': simulation,
        'trajectory.external.simulation': simulation,
        'trajectory.external.takeoff_height': gates['mission']['flight_height'],
    }
    clock_parameters = {'use_sim_time': simulation}
    actions = [
        LogInfo(msg=f'[gap_flight] simulation={simulation}, use_sim_time={simulation}, '
                    f'controller_kind={controller_kind}, trajectory.type=external'),
        LogInfo(msg=f'[gap_flight] controller_params={controller_file}; gap_params={gap_file}'),
        Node(package='px4ctrl', executable='px4ctrl_node', name='px4ctrl_node', output='screen',
             parameters=[controller_file, controller_overrides]),
        Node(package='px4ctrl', executable='px4ctrlrate_node', name='px4ctrlrate_node', output='screen',
             parameters=[config_file('px4ctrl', RATECTRL_PARAMS), clock_parameters]),
        Node(package='gap_planner', executable='gap_planner_node', name='gap_planner', output='screen',
             parameters=[gap_file, model]),
        Node(package='gap_planner', executable='gap_trajectory_sender', name='gap_trajectory_sender',
             output='screen', parameters=[gap_file, clock_parameters]),
    ]
    if RECORD_DATA:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory('flight_data_recorder'))
                / 'launch' / 'record.launch.py'))))
    if simulation:
        actions.append(Node(package='quadsim_mujoco', executable='quadsim_node', name='quadsim_node',
            output='screen', parameters=[config_file('quadsim_mujoco', SIM_PARAMS),
                {'initial_position': SIM_INITIAL_POSITION, 'gap_scene_file': gap_file,
                 'log_directory': str(Path(SIM_LOG_ROOT) / time.strftime('%Y%m%d_%H%M%S')),
                 'headless': HEADLESS}]))
    if RVIZ:
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', config_file('gap_planner', RVIZ_CONFIG)],
            parameters=[clock_parameters]))
    return LaunchDescription(actions)
