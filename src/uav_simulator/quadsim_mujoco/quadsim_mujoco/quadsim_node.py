import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_srvs.srv import Empty
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from rclpy.duration import Duration
import math
import mujoco.viewer # version = 3.2.3
from ament_index_python.packages import get_package_share_directory
import os
import csv
import glfw
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import re
from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleLocalPosition
from px4_msgs.msg import ActuatorMotors
from px4debug_msgs.msg import Px4ctrlDebug
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

from .quad import QUAD
from .quad import Control_t
from .quad import PX4CtrlDebug_t

# sun: 仿真节点连接 ROS 2 控制器与 MuJoCo：接收电机指令、施加物理模型外力，
# sun: 再把 MuJoCo 真值包装成 PX4 传感器消息，并同步完成显示和 CSV 记录。

# 日志配置
LOG_FOLDER = "/home/sun/ros2_controller/music-drone-225/sim_log"
FILE_PREFIX = "log_data"  # 日志文件前缀
FILE_SUFFIX = ".csv"       # 日志文件后缀
# 可视化颜色配置（RGBA格式，范围0-1）
COLOR_GREEN = (0, 1, 0, 1)
COLOR_RED = (1, 0, 0, 1)
COLOR_BLUE = (0.118, 0.565, 1, 1)

PI = math.pi
deg2rad = lambda x: x / 180.0 * PI
rad2deg = lambda x: x / PI * 180.0
rad2rpm = lambda x: x / (2*PI) * 60

import time
from typing import Dict

def vec_to_quaternion(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    计算从起始向量u旋转到目标向量v的旋转四元数 (w, x, y, z)
    :param u: 起始三维向量，numpy数组，shape=(3,)
    :param v: 目标三维向量，numpy数组，shape=(3,)
    :return: 单位旋转四元数，numpy数组，shape=(4,)，顺序为 [w, x, y, z]
    """
    # 1. 类型校验：确保是三维向量
    assert u.shape == (3,) and v.shape == (3,), "向量必须是三维向量 (shape=(3,))"
    
    # 2. 单位化向量（核心：旋转与向量长度无关）
    norm_u = np.linalg.norm(u)
    norm_v = np.linalg.norm(v)
    if norm_u < 1e-8 or norm_v < 1e-8:
        raise ValueError("起始向量/目标向量不能是零向量！")
    u_unit = u / norm_u
    v_unit = v / norm_v

    # 3. 计算点乘和叉乘
    dot_product = np.dot(u_unit, v_unit)  # cosθ
    cross_product = np.cross(u_unit, v_unit)  # u×v，未单位化的旋转轴
    sin_theta = np.linalg.norm(cross_product)  # sinθ

    # sun: 同向和反向共线时叉积退化，必须单独处理才能避免旋转轴除零。
    if dot_product > 1 - 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    
    # 5. 处理特殊情况2：向量反向共线 → 旋转180°，构造垂直轴
    if dot_product < -1 + 1e-8:
        # 构造任意一个垂直于u_unit的向量作为旋转轴
        if abs(u_unit[0]) < abs(u_unit[1]) and abs(u_unit[0]) < abs(u_unit[2]):
            axis = np.array([1.0, 0.0, 0.0])
        elif abs(u_unit[1]) < abs(u_unit[2]):
            axis = np.array([0.0, 1.0, 0.0])
        else:
            axis = np.array([0.0, 0.0, 1.0])
        axis = np.cross(u_unit, axis)
        axis = axis / np.linalg.norm(axis)
        # 180°旋转的四元数：cos(90°)=0，sin(90°)=1 → q=(0, ax, ay, az)
        return np.array([0.0, axis[0], axis[1], axis[2]])

    # 6. 正常情况：计算旋转轴+旋转四元数
    axis = cross_product / sin_theta  # 单位化旋转轴
    theta_half = np.arccos(np.clip(dot_product, -1.0, 1.0)) / 2.0  # θ/2，clip防止浮点误差导致超界
    w = np.cos(theta_half)
    x, y, z = axis * np.sin(theta_half)

    return np.array([w, x, y, z]) / np.linalg.norm(np.array([w, x, y, z]))  # 返回单位四元数

class TicToc:
    """进阶版tic/toc计时器，支持多命名计时器、嵌套计时"""
    def __init__(self):
        self.timers: Dict[str, float] = {}  # 存储多个计时器的开始时间

    def tic(self, timer_name: str = "default") -> None:
        """
        开始计时（可指定计时器名称）
        :param timer_name: 计时器名称，默认"default"
        """
        self.timers[timer_name] = time.perf_counter()

    def toc(self, timer_name: str = "default", print_msg: str = None) -> float:
        """
        结束计时并输出耗时
        :param timer_name: 对应tic的计时器名称
        :param print_msg: 自定义输出提示，默认格式："[timer_name] 耗时: xxx"
        :return: 耗时（秒）
        """
        if timer_name not in self.timers:
            raise KeyError(f"未找到名为'{timer_name}'的计时器，请先调用tic('{timer_name}')！")
        
        end_time = time.perf_counter()
        elapsed_time = end_time - self.timers[timer_name]
        
        # 生成默认提示文字
        if print_msg is None:
            print_msg = f"[{timer_name}] 耗时: "
        
        # 格式化输出
        # if elapsed_time < 1e-6:
        #     print(f"{print_msg}{elapsed_time * 1e9:.2f} 纳秒")
        # elif elapsed_time < 1e-3:
        #     print(f"{print_msg}{elapsed_time * 1e6:.2f} 微秒")
        # elif elapsed_time < 1:
        print(f"{print_msg}{elapsed_time * 1e3:.2f} 毫秒",flush=True)
        # else:
        #     print(f"{print_msg}{elapsed_time:.4f} 秒")
        
        return elapsed_time

    def reset(self, timer_name: str = None) -> None:
        """
        重置计时器
        :param timer_name: 要重置的计时器名称，None则重置所有
        """
        if timer_name is None:
            self.timers.clear()
        elif timer_name in self.timers:
            del self.timers[timer_name]
            
class IbvsDrawData:
    def __init__(self):
        self.nt_I = np.array([0.0, 0.0, 1.0])
        self.v_I_des = np.array([0.0, 0.0, 0.0])
        self.v_B_des = np.zeros(3)
        self.e_G = np.zeros(2)
            
class QuadSimNode(Node):
    class LogModule:
        # sun: 日志模块把仿真真值和控制参考展平成固定列顺序，便于 PlotJuggler 或脚本直接读取。
        class LogData:
            def __init__(self):
                self.timestamp_us = 0
                self.pos_I = np.zeros(3)
                self.vel_I = np.zeros(3)
                self.acc_I = np.zeros(3)
                self.acc_B = np.zeros(3)
                self.euler = np.zeros(3)
                self.omega_B = np.zeros(3)
                self.quat = np.zeros(4)
                self.motor_rpm = np.zeros(4)
                self.servo_pos_rad = np.zeros(2)
                self.servo_vel_rad = np.zeros(2)
                self.ct = np.zeros(4)
                self.cq = np.zeros(4)
                self.alphai = np.zeros(9)
                self.betai = np.zeros(9)
                self.Si = np.zeros(9)
                self.ri = [np.zeros(3)]*9
                self.cmaci = np.zeros(9)
                self.Cni = np.zeros(9)
                self.Cai = np.zeros(9)
                self.Cmi = np.zeros(9)
                self.Ni = np.zeros(9)
                self.Ai = np.zeros(9)
                self.Mi = np.zeros(9)
                self.F_total_I = np.zeros(3)
                self.M_total_I = np.zeros(3)
                self.ref_pos_I = np.zeros(3)
                self.ref_vel_I = np.zeros(3)
                self.ref_acc_I = np.zeros(3)
                self.des_quat = np.zeros(4)
                self.des_omega_B = np.zeros(3)
                self.des_vel_B = np.zeros(3)
                self.e_G = np.zeros(2)
                self.nt_I = np.zeros(3)
                self.target_pos_I = np.zeros(3)
                self.target_vel_I = np.zeros(3)
                self.target_acc_I = np.zeros(3)
                self.concact = 0
                
        def __init__(self, log_file):
            # 1. 定义表头 
            header = ["timestamp_us",
                    "pos_I_x", "pos_I_y", "pos_I_z",
                    "vel_I_x", "vel_I_y", "vel_I_z",
                    "acc_I_x", "acc_I_y", "acc_I_z",
                    "acc_B_x", "acc_B_y", "acc_B_z",
                    "euler_roll", "euler_pitch", "euler_yaw",
                    "omega_B_x", "omega_B_y", "omega_B_z",
                    "quat_w", "quat_x", "quat_y", "quat_z",
                    "motor_rpm_1", "motor_rpm_2", "motor_rpm_3", "motor_rpm_4",
                    "ct_1", "ct_2", "ct_3", "ct_4",
                    "cq_1", "cq_2", "c1_3", "cq_4",
                    "F_total_I_x", "F_total_I_y", "F_total_I_z",
                    "M_total_I_x", "M_total_I_y", "M_total_I_z",
                    "ref_pos_I_x", "ref_pos_I_y", "ref_pos_I_z",
                    "ref_vel_I_x", "ref_vel_I_y", "ref_vel_I_z",
                    "ref_acc_I_x", "ref_acc_I_y", "ref_acc_I_z",
                    "des_quat_w", "des_quat_x", "des_quat_y", "des_quat_z",
                    "des_omega_B_x", "des_omega_B_y", "des_omega_B_z",
                    "des_vel_B_x", "des_vel_B_y", "des_vel_B_z"]
            
            self.log_data = self.LogData()
            self.log_file = log_file
            # 创建CSV写入器
            self.log_writer = csv.writer(self.log_file)
            # 3. 判断文件是否为空，空则写入表头（仅首次运行时执行）
            self.log_file.seek(0)  # 移动文件指针到开头
            if not self.log_file.read(1):  # 读取1个字符，为空则表示文件是新的
                self.log_writer.writerow(header)
            
        def append_row_data(self, quad: QUAD, px4ctrldebug:PX4CtrlDebug_t, coord_sys='ENU'):
            # sun: 默认记录 ENU；选择其他坐标约定时统一翻转 y/z 及对应四元数和角速度分量。
            state = quad.state
            if coord_sys == 'ENU':
                sgn = 1
            else:
                sgn = -1
                
            self.log_data.timestamp_us = quad.now_time * 1e6
                    
            # 位置处理
            self.log_data.pos_I[0] = state.pos[0]
            self.log_data.pos_I[1] = sgn*state.pos[1]
            self.log_data.pos_I[2] = sgn*state.pos[2]
            
            # 速度处理
            self.log_data.vel_I[0] = state.vel[0]
            self.log_data.vel_I[1] = sgn*state.vel[1]
            self.log_data.vel_I[2] = sgn*state.vel[2]
            
            # 加速度处理
            self.log_data.acc_I[0] = state.acc[0]
            self.log_data.acc_I[1] = sgn*state.acc[1]
            self.log_data.acc_I[2] = sgn*state.acc[2]
            self.log_data.acc_B[0] = state.acc_B[0]
            self.log_data.acc_B[1] = sgn*state.acc_B[1]
            self.log_data.acc_B[2] = sgn*state.acc_B[2]
            
            # 四元数处理
            self.log_data.quat[0] = state.quat[0]
            self.log_data.quat[1] = state.quat[1]
            self.log_data.quat[2] = sgn*state.quat[2]
            self.log_data.quat[3] = sgn*state.quat[3]
            
            # 欧拉角处理
            euler = R.from_quat(state.quat).as_euler('xyz')
            self.log_data.euler[0] = euler[0]
            self.log_data.euler[1] = sgn*euler[1]
            self.log_data.euler[2] = sgn*euler[2]
            
            # 角速度处理
            self.log_data.omega_B[0] = state.omega[0]
            self.log_data.omega_B[1] = sgn*state.omega[1]
            self.log_data.omega_B[2] = sgn*state.omega[2]
            
            # 电机转速处理
            actuator_state = quad.actuator_state
            motor_rpm = rad2rpm(actuator_state.motor_vel_rad)
            for i in range(4):
                self.log_data.motor_rpm[i] = motor_rpm[i]
            
            # 电机推力力矩系数
            for i in range(4):
                self.log_data.ct[i] = quad.quad_debug.ct[i]
                self.log_data.cq[i] = quad.quad_debug.cm[i]
            
            # 合力
            self.log_data.F_total_I[0] = quad.quad_debug.total_force_I[0]
            self.log_data.F_total_I[1] = sgn*quad.quad_debug.total_force_I[1]
            self.log_data.F_total_I[2] = sgn*quad.quad_debug.total_force_I[2]
            self.log_data.M_total_I[0] = quad.quad_debug.total_moment_I[0]
            self.log_data.M_total_I[1] = sgn*quad.quad_debug.total_moment_I[1]
            self.log_data.M_total_I[2] = sgn*quad.quad_debug.total_moment_I[2]
            
            # 参考状态
            self.log_data.ref_pos_I[0] = px4ctrldebug.ref_p_I[0]
            self.log_data.ref_pos_I[1] = sgn*px4ctrldebug.ref_p_I[1]
            self.log_data.ref_pos_I[2] = sgn*px4ctrldebug.ref_p_I[2]
            self.log_data.ref_vel_I[0] = px4ctrldebug.ref_v_I[0]
            self.log_data.ref_vel_I[1] = sgn*px4ctrldebug.ref_v_I[1]
            self.log_data.ref_vel_I[2] = sgn*px4ctrldebug.ref_v_I[2]
            self.log_data.ref_acc_I[0] = px4ctrldebug.ref_a_I[0]
            self.log_data.ref_acc_I[1] = sgn*px4ctrldebug.ref_a_I[1]
            self.log_data.ref_acc_I[2] = sgn*px4ctrldebug.ref_a_I[2]
            
            # 期望值
            self.log_data.des_quat[0] = px4ctrldebug.des_q[0]
            self.log_data.des_quat[1] = px4ctrldebug.des_q[1]
            self.log_data.des_quat[2] = sgn*px4ctrldebug.des_q[2]
            self.log_data.des_quat[3] = sgn*px4ctrldebug.des_q[3]
            self.log_data.des_omega_B[0] = px4ctrldebug.des_ome_B[0]
            self.log_data.des_omega_B[1] = sgn*px4ctrldebug.des_ome_B[1]
            self.log_data.des_omega_B[2] = sgn*px4ctrldebug.des_ome_B[2]
            
            row = [
                self.log_data.timestamp_us,
                self.log_data.pos_I[0], self.log_data.pos_I[1], self.log_data.pos_I[2],
                self.log_data.vel_I[0], self.log_data.vel_I[1], self.log_data.vel_I[2],
                self.log_data.acc_I[0], self.log_data.acc_I[1], self.log_data.acc_I[2],
                self.log_data.acc_B[0], self.log_data.acc_B[1], self.log_data.acc_B[2],
                self.log_data.euler[0], self.log_data.euler[1], self.log_data.euler[2],
                self.log_data.omega_B[0], self.log_data.omega_B[1], self.log_data.omega_B[2],
                self.log_data.quat[0], self.log_data.quat[1], self.log_data.quat[2], self.log_data.quat[3],
                self.log_data.motor_rpm[0], self.log_data.motor_rpm[1], self.log_data.motor_rpm[2], self.log_data.motor_rpm[3],
                self.log_data.ct[0], self.log_data.ct[1], self.log_data.ct[2], self.log_data.ct[3],
                self.log_data.cq[0], self.log_data.cq[1], self.log_data.cq[2], self.log_data.cq[3],
            ] + \
                self.log_data.F_total_I.tolist() + \
                self.log_data.M_total_I.tolist() + \
                self.log_data.ref_pos_I.tolist() + \
                self.log_data.ref_vel_I.tolist() + \
                self.log_data.ref_acc_I.tolist() + \
                self.log_data.des_quat.tolist() + \
                self.log_data.des_omega_B.tolist() + \
                self.log_data.des_vel_B.tolist()
                
            self.log_writer.writerow(row)
            
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        
        # sun: 先与顶层控制节点双向握手，再创建参数和数据通道，避免控制器使用未配置的模型。
        node1_name = "px4ctrl_node"; # px4ctrl_node
        node2_name = name # quadsim_node
        self.handshake_server = self.create_service(
            Empty,
            "/"+node2_name+"/handshake",
            self.handshake_callback
        )
        self.get_logger().info("\033[33m节点"+node2_name+"：握手服务端已启动，等待节点"+node1_name+"上线...\033[0m")
        self.handshake_client = self.create_client(Empty, "/"+node1_name+"/handshake")
        while rclpy.ok():
            if self.handshake_client.wait_for_service(timeout_sec=1.0):
                # 发送握手请求
                request = Empty.Request()
                future = self.handshake_client.call_async(request)
                # 等待响应（确认节点在线）
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    self.get_logger().info("\033[32m节点"+node2_name+"：与节点"+node1_name+"握手成功！开始执行核心逻辑...\033[0m")
                    break
            self.get_logger().warn("\033[33m节点"+node2_name+"：未检测到节点"+node1_name+"，继续等待...\033[0m")
            time.sleep(1) # 避免循环过快
            
        # 创建参数客户端
        self.parameter_client = self.create_client(GetParameters,'/px4ctrl_node/get_parameters')
        # 等待参数服务器启动
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the parameter service. Exiting.")
                return
            self.get_logger().info("Waiting for the parameter service to start...")
            
        # sun: 传感器和执行器使用深度 1 的 best-effort QoS，实时性优先于补发旧样本。
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        # 创建发布者
        self.sensor_combined_publisher = self.create_publisher(SensorCombined, '/fmu/out/sensor_combined', qos_profile)
        self.vehicle_attitude_publisher = self.create_publisher(VehicleAttitude, '/fmu/out/vehicle_attitude', qos_profile)
        self.vehicle_local_position_publisher = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile)
        # 创建订阅者
        self.px4ctrldebug_subscription = self.create_subscription(
            Px4ctrlDebug,
            '/debugPx4/ctrl',
            self.px4ctrldebug_callback,
            1
        )
        self.actuator_motors_subscription = self.create_subscription(
            ActuatorMotors,
            '/fmu/in/actuator_motors',
            self.actuator_motors_callback,
            qos_profile
        )
        
        
        self.noise = np.random.default_rng()

        # 回调用到的参数
        self.rc2speed_a = 0.0
        self.rc2speed_b = 0.0
        self.rc2speed_c = 0.0
        # 控制参数初始化
        self.control = Control_t()
        self.last_control = Control_t()
        
        self.px4ctrldebug = PX4CtrlDebug_t()
        self.start_time = time.time()
        
    # 握手服务端的回调（仅需返回成功）
    def handshake_callback(self, request, response):
        # self.get_logger().info("收到握手请求，确认在线！")
        return response
    
    def sens_topic_pub(self, quad: QUAD):
        # sun: 内部真值为 FLU/ENU，发布 PX4 SensorCombined 时翻转 y/z 生成 FRD 符号。
        msg = SensorCombined()
        state = quad.state
        msg.timestamp = round(quad.now_time * 1e6)
        # msg.timestamp = self.get_clock().now().nanoseconds // 1000
        
        # 角速度处理（FRD坐标系）
        if quad.param.noise.gyro_is_valid:
            msg.gyro_rad[0] = state.omega[0] + self.noise.normal(0.0,quad.param.noise.gyro_std[0])
            msg.gyro_rad[1] = -state.omega[1] + self.noise.normal(0.0,quad.param.noise.gyro_std[1])
            msg.gyro_rad[2] = -state.omega[2] + self.noise.normal(0.0,quad.param.noise.gyro_std[2])
        else:
            msg.gyro_rad[0] = state.omega[0]
            msg.gyro_rad[1] = -state.omega[1]
            msg.gyro_rad[2] = -state.omega[2]
        
        # 加速度处理
        if quad.param.noise.accel_is_valid:
            msg.accelerometer_m_s2[0] = state.acc[0] + self.noise.normal(0.0,quad.param.noise.accel_std[0])
            msg.accelerometer_m_s2[1] = - state.acc[1] + self.noise.normal(0.0,quad.param.noise.accel_std[1])
            msg.accelerometer_m_s2[2] = - state.acc[2] + self.noise.normal(0.0,quad.param.noise.accel_std[2])
        else:
            msg.accelerometer_m_s2[0] = state.acc[0]
            msg.accelerometer_m_s2[1] = - state.acc[1]
            msg.accelerometer_m_s2[2] = - state.acc[2]
        
        self.sensor_combined_publisher.publish(msg)
    
    def att_topic_pub(self, quad: QUAD):
        # sun: MuJoCo 和工程内部四元数为 wxyz；PX4 消息同为 wxyz，但坐标转换需翻转 y/z。
        msg = VehicleAttitude()
        state = quad.state
        msg.timestamp = round(quad.now_time * 1e6)
        
        # 四元数处理（FRD坐标系）
        msg.q[0] = state.quat[0]
        msg.q[1] = state.quat[1]
        msg.q[2] = -state.quat[2]
        msg.q[3] = -state.quat[3]
        
        self.vehicle_attitude_publisher.publish(msg)
        
    def local_pose_topic_pub(self, quad: QUAD):
        # sun: 位置和速度从 ENU 转为 PX4 NED；仿真真值始终有效，因此所有有效标志置 true。
        msg = VehicleLocalPosition()
        state = quad.state
        msg.timestamp = round(quad.now_time * 1e6)
        
        # 位置处理（NED坐标系）
        msg.x = state.pos[0]
        msg.y = -state.pos[1]
        msg.z = -state.pos[2]
        
        # 速度处理
        msg.vx = state.vel[0]
        msg.vy = -state.vel[1]
        msg.vz = -state.vel[2]
        
        # 设置标志位
        msg.delta_xy = [0.0, 0.0]
        msg.delta_z = 0.0
        msg.delta_vxy = [0.0, 0.0]
        msg.delta_vz = 0.0
        msg.xy_valid = True
        msg.z_valid = True
        msg.v_xy_valid = True
        msg.v_z_valid = True
        
        self.vehicle_local_position_publisher.publish(msg)
        
    def px4ctrldebug_callback(self, msg: Px4ctrlDebug):
        self.px4ctrldebug.ref_p_I = np.array([msg.ref_p_x, -msg.ref_p_y, -msg.ref_p_z])
        self.px4ctrldebug.ref_v_I = np.array([msg.ref_v_x, -msg.ref_v_y, -msg.ref_v_z])
        self.px4ctrldebug.ref_a_I = np.array([msg.ref_a_x, -msg.ref_a_y, -msg.ref_a_z])
        self.px4ctrldebug.des_q = np.array([msg.des_q_w, msg.des_q_x, -msg.des_q_y, -msg.des_q_z])
        self.px4ctrldebug.des_ome_B = np.array([msg.des_rate_x, -msg.des_rate_y, -msg.des_rate_z])
    
    def actuator_motors_callback(self, msg: ActuatorMotors):
        # sun: 控制器给出归一化油门，通过试验标定二次式恢复 rad/s；NaN 通道保持上一有效转速。
        self.last_control = self.control
        throttle_in = np.array(msg.control)
        
        for i in range(4):
            if np.isnan(throttle_in[i]):
                self.control.motor_vel_rad[i] = self.last_control.motor_vel_rad[i]
            else:
                speed_rad = (self.rc2speed_a * throttle_in[i]**2 + 
                             self.rc2speed_b * throttle_in[i] + 
                             self.rc2speed_c)
                self.control.motor_vel_rad[i] = speed_rad
        # if time.time() - self.start_time > 0.01:
        #     print("throttle_in=",throttle_in[0:4],flush=True)
        #     print("speed_rad=",self.control.motor_vel_rad,flush=True)
        #     self.start_time = time.time()
            
    def _get_parameters(self, parameter_names):
        # sun: 参数服务按单个名称同步等待最多两秒，并根据 ROS 参数类型提取 Python 标量。
        req = GetParameters.Request() # 请求实例
        req.names = [parameter_names] # 请求实例，参数名称表
        
        future = self.parameter_client.call_async(req) # 异步操作 实例
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
        if future.done():
            try:
                response = future.result()
                # 检查是否成功获取参数值
                if response is not None:
                    param = response.values[0]
                    if param.type == ParameterType.PARAMETER_DOUBLE:
                        return param.double_value
                    elif param.type == ParameterType.PARAMETER_INTEGER:
                        return param.integer_value
                    elif param.type == ParameterType.PARAMETER_BOOL:
                        return param.bool_value
                    # 其他类型处理...
                    else:
                        raise TypeError(f"Unsupported parameter type: {param.type}")
                else:
                    self.get_logger().warn(f"{str(parameter_names)} 返回空响应")
            except Exception as e:
                self.get_logger().error(f"{str(parameter_names)} 服务调用异常: {str(e)}")
        else:
            self.get_logger().warn(f"{str(parameter_names)} 请求超时，未收到响应")
        return None  # 失败时返回None
 
    def parameter_set(self, quad: QUAD):
        # sun: 飞行器/控制共有参数从 px4ctrl_node 拉取，仿真专用参数从本节点 YAML 读取。
        # try: 
        # 获取参数
        quad.param.gra = self._get_parameters('gra')
        quad.param.uav.mass = self._get_parameters('uav.mass')
        quad.param.uav.J = np.diag([
            self._get_parameters('uav.Jvx'),
            self._get_parameters('uav.Jvy'),
            self._get_parameters('uav.Jvz')
        ])
        quad.param.uav.l = self._get_parameters('uav.l')
        quad.param.uav.rp = self._get_parameters('uav.rp')
        quad.param.uav.beta_deg = self._get_parameters('uav.beta_deg')
        quad.param.motor.cq0= self._get_parameters('motor.cq0')
        quad.param.motor.ct0 = self._get_parameters('motor.ct0')
        quad.param.motor.Cq_a = self._get_parameters('motor.Cq_a')
        quad.param.motor.Cq_b = self._get_parameters('motor.Cq_b')
        quad.param.motor.Cq_c = self._get_parameters('motor.Cq_c')
        quad.param.motor.Ct_a = self._get_parameters('motor.Ct_a')
        quad.param.motor.Ct_b = self._get_parameters('motor.Ct_b')
        quad.param.motor.Ct_c = self._get_parameters('motor.Ct_c')
        quad.param.motor.rc2speed_a = self._get_parameters('motor.rc2speed_a')
        quad.param.motor.rc2speed_b = self._get_parameters('motor.rc2speed_b')
        quad.param.motor.rc2speed_c = self._get_parameters('motor.rc2speed_c')
        self.rc2speed_a = quad.param.motor.rc2speed_a
        self.rc2speed_b = quad.param.motor.rc2speed_b
        self.rc2speed_c = quad.param.motor.rc2speed_c
        quad.param.motor.u_min = self._get_parameters('motor.u_min')
        quad.param.motor.u_max = self._get_parameters('motor.u_max')
        quad.param.motor.speed_min = quad.param.motor.rc2speed_c
        quad.param.motor.speed_max = quad.param.motor.rc2speed_a + quad.param.motor.rc2speed_b + quad.param.motor.rc2speed_c
        quad.param.aero.rho = self._get_parameters('aero.rho')
        quad.param.aero.kdx = self._get_parameters('aero.kdx')
        quad.param.aero.kdy = self._get_parameters('aero.kdy')
        quad.param.aero.kdz = self._get_parameters('aero.kdz')
        
        # 从yaml获取参数
        self.declare_parameter('sim_freq_max', 400.0)
        quad.param.sim_freq_max = self.get_parameter('sim_freq_max').get_parameter_value().double_value
        self.declare_parameter('motor.Tm_a', 0.0)
        self.declare_parameter('motor.Tm_b', 0.0)
        self.declare_parameter('motor.Jm', 0.0)
        quad.param.motor.Tm_a = self.get_parameter('motor.Tm_a').get_parameter_value().double_value
        quad.param.motor.Tm_b = self.get_parameter('motor.Tm_b').get_parameter_value().double_value
        quad.param.motor.Jm = self.get_parameter('motor.Jm').get_parameter_value().double_value
        
        self.declare_parameter('noise.wind_is_valid', False)
        self.declare_parameter('noise.wind_mean', 0.01)
        self.declare_parameter('noise.wind_height', 0.0)
        quad.param.noise.wind_is_valid = self.get_parameter('noise.wind_is_valid').get_parameter_value().bool_value
        quad.param.noise.wind_mean = np.max([self.get_parameter('noise.wind_mean').get_parameter_value().double_value,0.01])
        quad.param.noise.wind_height = self.get_parameter('noise.wind_height').get_parameter_value().double_value

        self.declare_parameter('noise.gyro_is_valid', False)
        self.declare_parameter('noise.gyro_stddev_x', 0.0)
        self.declare_parameter('noise.gyro_stddev_y', 0.0)
        self.declare_parameter('noise.gyro_stddev_z', 0.0)
        self.declare_parameter('noise.accel_is_valid', False)
        self.declare_parameter('noise.accel_stddev_x', 0.0)
        self.declare_parameter('noise.accel_stddev_y', 0.0)
        self.declare_parameter('noise.accel_stddev_z', 0.0)
        quad.param.noise.accel_is_valid = self.get_parameter('noise.accel_is_valid').get_parameter_value().bool_value
        quad.param.noise.accel_std = np.array([
            self.get_parameter('noise.accel_stddev_x').get_parameter_value().double_value,
            self.get_parameter('noise.accel_stddev_y').get_parameter_value().double_value,
            self.get_parameter('noise.accel_stddev_z').get_parameter_value().double_value
        ])
        quad.param.noise.gyro_is_valid = self.get_parameter('noise.gyro_is_valid').get_parameter_value().bool_value
        quad.param.noise.gyro_std = np.array([
            self.get_parameter('noise.gyro_stddev_x').get_parameter_value().double_value,
            self.get_parameter('noise.gyro_stddev_y').get_parameter_value().double_value,
            self.get_parameter('noise.gyro_stddev_z').get_parameter_value().double_value
        ])
        
        self.declare_parameter('visual.arrow_display', False)
        self.declare_parameter('visual.arrow_width', 0.08)
        self.declare_parameter('visual.arrow_aspect_ratio', 100.0)
        self.declare_parameter('visual.traj_line_display', False)
        self.declare_parameter('visual.traj_line_length', 1000)
        self.declare_parameter('visual.traj_line_width', 5.0)
        quad.param.visual.arrow_display = self.get_parameter('visual.arrow_display').get_parameter_value().bool_value
        quad.param.visual.arrow_width = self.get_parameter('visual.arrow_width').get_parameter_value().double_value
        quad.param.visual.arrow_aspect_ratio = self.get_parameter('visual.arrow_aspect_ratio').get_parameter_value().double_value
        quad.param.visual.traj_line_display = self.get_parameter('visual.traj_line_display').get_parameter_value().bool_value
        quad.param.visual.traj_line_length = self.get_parameter('visual.traj_line_length').get_parameter_value().integer_value
        quad.param.visual.traj_line_width = self.get_parameter('visual.traj_line_width').get_parameter_value().double_value
            
            # print("quad.param.motor.u_min:",quad.param.motor.u_min,flush=True)
            
        # except Exception as e:
        #     self.get_logger().warn('Unknown parameters are set : %r' % (e,))
        

def init_mujoco(model: mujoco._structs.MjModel, quad: QUAD, node: QuadSimNode):
    # sun: 把 YAML 中的质量、惯量、重力、步长和机臂几何写入已加载的 MuJoCo 模型。
    quad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quad")
    # 配置
    model.opt.timestep = 1.0 / quad.param.sim_freq_max
    # 环境
    model.opt.gravity = [0, 0, -quad.param.gra]
    # 惯性
    model.body(quad_id).mass = quad.param.uav.mass
    model.body(quad_id).inertia = np.diag(quad.param.uav.J)
    model.body(quad_id).ipos = np.array([0.0, 0.0, 0.0])    # 这条设置并没有起左右，目前没找到原因
    # 电机位置
    motor1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "motor1")
    motor2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "motor2")
    motor3_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "motor3")
    motor4_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "motor4")
    #         x
    #   (↻)1  ↑  2(↺)
    #       ╲β| ╱ 
    #        ╲│╱
    # y ← ——— ⊙ z     
    #        ╱ ╲
    #       ╱   ╲
    #   (↺)4    3(↻)  
    beta = deg2rad(quad.param.uav.beta_deg) 
    l = quad.param.uav.l
    model.body(motor1_id).pos = [l*np.cos(beta), l*np.sin(beta), 0]
    model.body(motor2_id).pos = [l*np.cos(beta), -l*np.sin(beta), 0]
    model.body(motor3_id).pos = [-l*np.cos(beta),-l*np.sin(beta), 0]
    model.body(motor4_id).pos = [-l*np.cos(beta), l*np.sin(beta), 0]
    # 传感器位置
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "quad_sensor_site")
    model.site(sensor_id).pos = [0.0, 0.0, 0.0]
    
    
def time_sleep(elapsed): # time.sleep()的精确版本
    # sun: 忙等待仅用于需要微秒级补偿的实验路径，正常主循环使用 time.sleep() 以降低 CPU 占用。
    delay_mark = time.time()
    offset = 0
    while offset < elapsed-3.5e-6:
        offset = time.time() - delay_mark
        
def quat_mujoco2scipy(q_wxyz):
    # sun: MuJoCo 使用 wxyz，SciPy Rotation 使用 xyzw；所有可视化转换统一经过这两个函数。
    q_xyzw = [q_wxyz[1],q_wxyz[2],q_wxyz[3],q_wxyz[0]]
    return q_xyzw

def quat_scipy2mujoco(q_xyzw):
    q_wxyz = [q_xyzw[3],q_xyzw[0],q_xyzw[1],q_xyzw[2]]
    return q_wxyz

def main(args=None):
    # sun: 启动顺序为节点握手 -> 参数同步 -> 风场/悬停转速 -> MuJoCo 模型 -> 实时循环。
    rclpy.init(args=args)                            
    node = QuadSimNode("quadsim_node")             
    
    # 初始化四旋翼模型
    quad = QUAD()
    node.parameter_set(quad)
    
    # 风场设置
    quad.wind_model.reset(
        U0=quad.param.noise.wind_mean,       # 平均风速m/s
        height=quad.param.noise.wind_height,   # 飞行高度m
        dt=1/quad.param.sim_freq_max,          # 与动力学步长一致
        duration=200.0,  # 预生成n秒的风场数据
        loop=True       # 循环复用风场
    )
    
    # 设置初始状态
    hover_thrust = quad.param.uav.mass * quad.param.gra / 4.0
    hover_rad = math.sqrt(hover_thrust / quad.param.motor.ct0)
    quad.actuator_state.motor_vel_rad = np.full(4, hover_rad)
    quad.update_actuator_internal_state()
    
    # 读取mujoco模型
    package_path = get_package_share_directory('quadsim_mujoco')
    mjcf_path = os.path.join(package_path,'mjcf','quad.xml')
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    init_mujoco(model, quad, node)
    data = mujoco.MjData(model) # 用来存储仿真数据
    

    with mujoco.viewer.launch_passive(model, data) as viewer:
        quad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quad")
        
        # 修改相机参数
        viewer.cam.distance = 3.0      # 相机距离目标点距离
        viewer.cam.azimuth = 0        # 水平旋转角 (0-360度)
        viewer.cam.elevation = -35     # 俯仰角 (-90 到 90度)
        viewer.cam.lookat[:] = [0,0,5] # 目标点坐标 (x, y, z)
        
        last_time = time.time()
        quad.start_time = time.time()
        
        # 可视化设置
        # 箭头设置
        if quad.param.visual.arrow_display:
            arrow_num = 3
            arrow_width = quad.param.visual.arrow_width
            arrow_aspect_ratio = quad.param.visual.arrow_aspect_ratio
        else:
            arrow_num = 0
        viewer.user_scn.ngeom = arrow_num
        # 轨迹设置
        draw_traj_dt = 1.0 / quad.param.sim_freq_max * 5
        if quad.param.visual.traj_line_display:
            traj_num = 2
            traj_line_length = quad.param.visual.traj_line_length
            traj_line_width = quad.param.visual.traj_line_width
            traj_draw_last_time = time.time() - draw_traj_dt*1.2
        else:
            traj_num = 0
        traj_current_idx = arrow_num

        # 确保日志目录存在，避免首次运行时打开文件失败。
        os.makedirs(LOG_FOLDER, exist_ok=True)

        # 2. 拼接新日志文件的完整路径
        log_filename = os.path.join(LOG_FOLDER, f"{FILE_PREFIX}{FILE_SUFFIX}")
        print("LOG已保存:",log_filename,flush=True)
        # 2. 打开文件（w+模式：覆盖+读写，避免重复打开关闭）
        with open(log_filename, 'w+', newline='', encoding='utf-8') as log_file:
            log_module = node.LogModule(log_file)
            log_last_time = time.time()
            while rclpy.ok() and viewer.is_running():

                # sun: 每周期先处理最新 ROS 指令并计算外力，再推进电机和 MuJoCo 刚体动力学。
                now_time = time.time()    # time.time() 比 node.get_clock().now().nanosecends / 1e9 精确
                dt = now_time - last_time
                t = now_time - quad.start_time
                last_time = now_time
                rclpy.spin_once(node, timeout_sec=0) 
                quad.now_time = now_time
                # 施加力
                quad.set_input(node.control)
                quad_force_global, quad_moment_global = quad.get_body_force_moment()
                quad_xfrc_applied = data.xfrc_applied[quad_id]
                quad_xfrc_applied[0] = quad_force_global[0] # fx
                quad_xfrc_applied[1] = quad_force_global[1] # fy
                quad_xfrc_applied[2] = quad_force_global[2] # fz
                quad_xfrc_applied[3] = quad_moment_global[0] # tx
                quad_xfrc_applied[4] = quad_moment_global[1] # ty
                quad_xfrc_applied[5] = quad_moment_global[2] # tz
                                
                # 视觉更新
                motor1_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "prop1")
                motor2_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "prop2")
                motor3_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "prop3")
                motor4_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "prop4")
                model.geom(motor1_geom_id).quat = quat_scipy2mujoco(R.from_euler('xyz',[0, math.pi/2, quad.get_actuator_state().motor_pos_rad[0]/50]).as_quat())
                model.geom(motor2_geom_id).quat = quat_scipy2mujoco(R.from_euler('xyz',[0, math.pi/2, quad.get_actuator_state().motor_pos_rad[1]/50]).as_quat())
                model.geom(motor3_geom_id).quat = quat_scipy2mujoco(R.from_euler('xyz',[0, math.pi/2, -quad.get_actuator_state().motor_pos_rad[2]/50]).as_quat())
                model.geom(motor4_geom_id).quat = quat_scipy2mujoco(R.from_euler('xyz',[0, math.pi/2, -quad.get_actuator_state().motor_pos_rad[3]/50]).as_quat())
                
                # 模型积分
                quad.step(dt) # 执行器惯性环节
                mujoco.mj_step(model, data)
                
                # with viewer.lock():
                #     viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
                
                # 绘制坐标系箭头
                if quad.param.visual.arrow_display:
                    x_B = quad.state.Rbi.copy()[:,0]
                    q = vec_to_quaternion(np.array([0,0,1]),x_B)
                    mat = R.from_quat(np.array([q[1],q[2],q[3],q[0]])).as_matrix()
                    mujoco.mjv_initGeom(
                        viewer.user_scn.geoms[0],
                        type=mujoco.mjtGeom.mjGEOM_ARROW,
                        size=np.array([arrow_width, arrow_width, arrow_width*arrow_aspect_ratio]),
                        pos=quad.state.pos.copy(),
                        mat=mat.flatten(),  # 假设箭头为体轴系z轴,那么这里就是Rbi
                        rgba=np.array(COLOR_RED)
                    )
                    y_B = quad.state.Rbi.copy()[:,1]
                    q = vec_to_quaternion(np.array([0,0,1]),y_B)
                    mat = R.from_quat(np.array([q[1],q[2],q[3],q[0]])).as_matrix()
                    mujoco.mjv_initGeom(
                        viewer.user_scn.geoms[1],
                        type=mujoco.mjtGeom.mjGEOM_ARROW,
                        size=np.array([arrow_width, arrow_width, arrow_width*arrow_aspect_ratio]),
                        pos=quad.state.pos.copy(),
                        mat=mat.flatten(),  # 假设箭头为体轴系z轴,那么这里就是Rbi
                        rgba=np.array(COLOR_GREEN)
                    )
                    z_B = quad.state.Rbi.copy()[:,2]
                    q = vec_to_quaternion(np.array([0,0,1]),z_B)
                    mat = R.from_quat(np.array([q[1],q[2],q[3],q[0]])).as_matrix()
                    mujoco.mjv_initGeom(
                        viewer.user_scn.geoms[2],
                        type=mujoco.mjtGeom.mjGEOM_ARROW,
                        size=np.array([arrow_width, arrow_width, arrow_width*arrow_aspect_ratio]),
                        pos=quad.state.pos.copy(),
                        mat=mat.flatten(),  # 假设箭头为体轴系z轴,那么这里就是Rbi
                        rgba=np.array(COLOR_BLUE)
                    )
                
                # 绘制轨迹
                if quad.param.visual.traj_line_display:
                    if quad.now_time - traj_draw_last_time >= draw_traj_dt:
                        max_geom_num = traj_line_length*traj_num 
                        # 当前位置轨迹
                        geom = viewer.user_scn.geoms[traj_current_idx]
                        mujoco.mjv_initGeom(
                            geom,  # 关键修改：用循环索引
                            type=mujoco.mjtGeom.mjGEOM_SPHERE,
                            size=[0.04, 0, 0],  # 小球半径
                            pos=quad.state.pos.copy(),
                            mat=np.eye(3).flatten(),
                            rgba=[0, 1, 1, 1]
                        )
                        mujoco.mjv_connector(
                            geom, 
                            mujoco.mjtGeom.mjGEOM_LINE, 
                            traj_line_width, 
                            quad.state.pos.copy(), 
                            quad.state.pos.copy()+quad.state.vel.copy()*draw_traj_dt)
                        # 期望位置轨迹
                        geom = viewer.user_scn.geoms[traj_current_idx+1]
                        mujoco.mjv_initGeom(
                            geom,
                            type=mujoco.mjtGeom.mjGEOM_SPHERE,
                            size=[0.04, 0, 0],  # 小球半径
                            pos=node.px4ctrldebug.ref_p_I.copy(),
                            mat=np.eye(3).flatten(),
                            rgba=[1, 0, 0, 1]
                        )
                        mujoco.mjv_connector(
                            geom, 
                            mujoco.mjtGeom.mjGEOM_LINE, 
                            traj_line_width, 
                            node.px4ctrldebug.ref_p_I.copy(), 
                            node.px4ctrldebug.ref_p_I.copy()+node.px4ctrldebug.ref_v_I.copy()*draw_traj_dt)
                        
                        if viewer.user_scn.ngeom >= max_geom_num+arrow_num:
                            if traj_current_idx >= max_geom_num+arrow_num:
                                traj_current_idx = arrow_num
                            else:
                                traj_current_idx += traj_num
                        else:
                            viewer.user_scn.ngeom += traj_num  # 没满100个时，正常累加计数
                            traj_current_idx += traj_num
                            
                        traj_draw_last_time = quad.now_time
                            

                viewer.sync()
                # sun: 积分完成后从 MuJoCo 传感器读取新状态，下一周期动力学和本周期消息均使用该快照。
                Rbi = R.from_quat(quat_mujoco2scipy(quad.state.quat)).as_matrix() 
                quad.state.Rbi = Rbi
                quad.state.omega = data.sensor("body_angvel").data.copy() # 体轴系
                quad.state.quat = data.sensor("body_quat").data.copy()
                quad.state.acc = Rbi @ data.sensor("body_linacc").data.copy()
                quad.state.acc_B = data.sensor("body_linacc").data.copy() #带有重力加速度的acc
                quad.state.vel = data.sensor("body_vel").data.copy()
                quad.state.vel_B = Rbi.T @ quad.state.vel.copy()
                quad.state.pos = data.sensor("body_pos").data.copy()

                node.sens_topic_pub(quad)
                node.att_topic_pub(quad)
                node.local_pose_topic_pub(quad)
                
                # 日志记录
                # if quad.now_time - log_last_time >= 0.005:
                log_module.append_row_data(quad, node.px4ctrldebug)
                    # log_last_time = quad.now_time
                
                # sun: 用剩余时间节流到模型步长；乘 1.2 预留调度开销，超时时不再额外睡眠。
                elapsed = model.opt.timestep - (time.time()-now_time)*1.2
                if elapsed > 0:  # 0.1s
                    # (0.015->0.0006) (0.05->0.0004（无录制)
                    time.sleep(np.clip(elapsed,0,None)) # -0.00015 
                    # time_sleep(elapsed)

                # 打印周期
                # if time.time() - print_last_time > 0.1:
                #     print("model.opt.timestep:",model.opt.timestep,flush=True)
                #     print("dt:",dt,flush=True)
                #     print("elapsed:",elapsed,flush=True)
                    # flap_euler = R.from_quat(quat_mujoco2scipy(model.body(flap_left_body_id).quat)).as_euler('xyz')
                    # print("flap_euler:",rad2deg(flap_euler),flush=True)
                    
                #     print("B:%.2f,%.2f,%.2f || %.2f,%.2f,%.2f" %
                #           (force_local[0],force_local[1],force_local[2],
                #            moment_local[0],moment_local[1],moment_local[2]),flush=True)
                #     print("I:%.2f,%.2f,%.2f || %.2f,%.2f,%.2f" %
                #           (force_global[0],force_global[1],force_global[2],
                #            moment_global[0],moment_global[1],moment_global[2]),flush=True)
                #     print("Ang:%.1f,%.1f,%.1f" % (euler_angles[0],euler_angles[1],euler_angles[2]))
                    # print("motor_rad[0]:",quad.get_actuator_state().motor_pos_rad[0],flush=True)
                    # print(quad.actuator_internal_state[0:4],flush=True)
                #     print(np.array(dura).mean())
                #     dura = []
                #     print("motor1_frc:",data.sensor("motor1_frc").data)
                    # print("elapsed:",elapsed,flush=True)
                    # print_last_time = time.time()
                    
        # except Exception as e:
        #     print("Exception:",e,flush=True)
        # except KeyboardInterrupt:
        #     pass
        # finally:
            # 释放MuJoCo对象
            del model
            del data
            node.destroy_node()                              # 销毁节点对象
            rclpy.shutdown()                                 # 关闭ROS2 Python接口
    
    
