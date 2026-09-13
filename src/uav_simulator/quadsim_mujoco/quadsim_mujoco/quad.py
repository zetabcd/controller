#! /usr/bin/env python3

import numpy as np
import math
import time
import random
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation
from .dryden_wind_field import DrydenWindField

# sun: 本文件实现控制器之外的四旋翼物理模型：电机一阶惯性、前进比桨模型、
# sun: 机体气动力以及四电机合力/合力矩。机体内部状态统一使用 ENU/FLU 约定。

# 定义常量
PI = math.pi
deg2rad = lambda x: x / 180.0 * PI
rad2deg = lambda x: x / PI * 180.0
rad2rpm = lambda x: x / (2*PI) * 60


class PX4CtrlDebug_t:
    # sun: 缓存顶层控制调试量，供仿真日志与参考轨迹可视化使用，不参与动力学积分。
    def __init__(self):
        self.ref_p_I = np.array([0.0, 0.0, 0.0])
        self.ref_v_I = np.array([0.0, 0.0, 0.0])
        self.ref_a_I = np.array([0.0, 0.0, 0.0])
        self.des_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.des_ome_B = np.array([0.0, 0.0, 0.0])
        self.v_dot_I = np.array([0.0, 0.0, 0.0])
        self.va_I = np.array([0.0, 0.0, 0.0])
        

class Control_t:
    def __init__(self):
        self.motor_vel_rad = np.zeros(4)

class AeroCoeff_t:
    def __init__(self, CL=0.0, CD=0.0, CN=0.0, CA=0.0, CM=0.0):
        self.CL = CL
        self.CD = CD
        self.CN = CN
        self.CA = CA
        self.CM = CM

class PARAM:
    # sun: 参数层次与控制器 YAML 基本一致，另包含仿真专用的执行器、噪声和显示参数。
    class UAV:
        def __init__(self):
            self.mass = 0.7311
            self.J = np.array([ [2.4813e-3, 0.0, 0.0],
                                [0.0, 2.6667e-3, 0.0],
                                [0.0, 0.0, 4.3365e-3]])
            self.l = 0.1125
            self.rp = 0.06475
            self.beta_deg = 55.4  # deg
            
    class MOTOR:
        def __init__(self):
            self.cq0 = 1.8346e-8
            self.ct0 = 1.7368e-6
            self.Cq_a = -0.0275
            self.Cq_b = 0.0017
            self.Cq_c = 0.0161
            self.Ct_a = -0.1861
            self.Ct_b = -0.1392
            self.Ct_c = 0.1991
            self.u_min = 0.0
            self.u_max = 13.3819
            self.rc2speed_a = -1059.68
            self.rc2speed_b = 3703.59
            self.rc2speed_c = 115.91
            self.speed_min = self.rc2speed_c
            self.speed_max = self.rc2speed_a + self.rc2speed_b + self.rc2speed_c
            self.hover_percentage = 0.3
            self.Tm_a = -2.2261e-6 
            self.Tm_b = 0.034629
            self.Jm = 6.0e-6
            
    class AERO:
        def __init__(self):
            self.rho = 1.225
            self.kdx = 0.26
            self.kdy = 0.28
            self.kdz = 0.42
            self.kh = 0.01
    
    class NOISE:
        def __init__(self):
            self.wind_is_valid = False
            self.wind_mean = 4.4
            self.wind_height = 10.0
            self.gyro_is_valid = False
            self.gyro_std = np.zeros(3)
            self.accel_is_valid = False
            self.accel_std = np.zeros(3)
            
            
    class VISUAL:
        def __init__(self):
            self.arrow_display = False
            self.arrow_width = 0.08
            self.arrow_aspect_ratio = 100.0
            self.traj_line_display = False
            self.traj_line_length = 1000
            self.traj_line_width = 5.0
            
    def __init__(self):
        self.sim_freq_max = 400.0
        self.gra = 9.805
        self.uav = self.UAV()
        self.motor = self.MOTOR()
        self.aero = self.AERO()
        self.noise = self.NOISE()
        self.visual = self.VISUAL()
              
class QUAD:
    # sun: QUAD 只计算执行器和外力，不直接积分六自由度刚体；位姿积分由 MuJoCo 完成。
    class State:
        def __init__(self):
            self.pos = np.zeros(3)  # 位置
            self.vel = np.zeros(3)  # 速度
            self.vel_B = np.zeros(3)
            self.acc = np.zeros(3)  # 加速度
            self.acc_B = np.zeros(3)
            self.omega = np.zeros(3)  # 角速度
            self.quat = np.array([1.0, 0.0, 0.0, 0.0])  # 四元数 (w, x, y, z)
            self.Rbi = np.diag([1.0,1.0,1.0])
            
    class ActuatorState:
        def __init__(self):
            self.motor_pos_rad = np.zeros(4)
            self.motor_vel_rad = np.zeros(4)

    class QuadDebug_t:
        def __init__(self):
            self.total_force_I = np.zeros(3)
            self.total_moment_I = np.zeros(3)
            self.ct = np.zeros(4)
            self.cm = np.zeros(4)
            
    def __init__(self):
        self.quad_debug = self.QuadDebug_t()
        # 初始化状态
        self.state = self.State()
        self.state.pos = np.array([0.0, 0.0, 0.0])
        ypr = np.array([deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)])
        rotation = Rotation.from_euler('zyx', ypr, degrees=False)
        self.state.q = rotation.as_quat()  # [x, y, z, w]
        self.actuator_state = self.ActuatorState()
        
        # 执行器状态
        self.actuator_internal_state = np.zeros(4)
        self.update_actuator_internal_state()
        
        # 噪声
        self.wind_model = DrydenWindField()
        
        # 输入
        self.input = Control_t()
        
        # 物理参数默认值
        self.param = PARAM()

        
        self.last_motor_rad = np.zeros(4)
        self.acc = np.zeros(3)
        # self.last_time = time.time()
        self.now_time = time.time()
        self.start_time = time.time()
        self.start_time2 = time.time()
        
    def update_actuator_internal_state(self):
        # 将执行器状态转换为内部状态数组
        self.actuator_internal_state[0:4] = self.actuator_state.motor_vel_rad
    
    def set_input(self, u: Control_t):
        # sun: 输入首先限制到电机可实现转速，真正转速随后经过一阶惯性环节逐步逼近。
        motor_vel_rad = np.clip(u.motor_vel_rad, self.param.motor.speed_min, self.param.motor.speed_max)
        self.input.motor_vel_rad = motor_vel_rad
        # if time.time() - self.start_time2 > 0.01:
        #     # print("u:", u.motor_vel_rad, flush=True)
        #     print("self.input.motor_vel_rad:", self.input.motor_vel_rad, flush=True)
        #     self.start_time2 = time.time()
        
    # 电机前进比模型
    def calc_motor_coefficient(self, v_b, vw_b):
        # sun: 轴向相对来流决定前进比 J，再由试验拟合多项式得到随工况变化的 ct/cm。
        V0 = v_b[2] - vw_b[2]
        motor_vel_rad = np.clip(self.actuator_state.motor_vel_rad, self.param.motor.speed_min, self.param.motor.speed_max)
        J = np.clip(math.pi * V0 / (motor_vel_rad * self.param.uav.rp), 0.0, None)
        motorCT = np.clip(self.param.motor.Ct_a * J**2 + self.param.motor.Ct_b * J + self.param.motor.Ct_c, self.param.motor.Ct_c/10, self.param.motor.Ct_c)
        motorCM = np.clip(self.param.motor.Cq_a * J**2 + self.param.motor.Cq_b * J + self.param.motor.Cq_c, self.param.motor.Cq_c/10, self.param.motor.Cq_c)
        motorct = motorCT / (math.pi**2/(4*self.param.aero.rho*self.param.uav.rp**4))
        motorcm = motorCM / (math.pi**2/(8*self.param.aero.rho*self.param.uav.rp**5))
        self.quad_debug.ct = motorct
        self.quad_debug.cm = motorcm
        return motorct, motorcm
    
    def get_body_force_moment(self):
        # sun: 风场输出和飞行速度均在机体系相减，得到相对气流；关闭风场时风速为零。
        if self.param.noise.wind_is_valid:
            u, v, w = self.wind_model.get_wind()
            vw_B = np.array([u, v, w])
        else:
            vw_B = np.zeros(3)
        v_B = self.state.vel_B
        ome = self.state.omega
        motorct, motorcm = self.calc_motor_coefficient(v_B, vw_B)
        # sun: 每个电机满足 T_i=ct_i·ω_i²，总推力沿机体 +z 方向。
        motor_vel_rad = self.actuator_state.motor_vel_rad
        motor_vel_rad_2 = motor_vel_rad**2
        motor_thrust = motorct * motor_vel_rad_2
        T = motor_thrust.sum()
        
        # sun: 滚转/俯仰力矩来自推力乘机臂，偏航来自桨反扭矩，G3 项补偿转子陀螺力矩。
        #         x
        #   (↻)1  ↑  2(↺)
        #       ╲β| ╱ 
        #        ╲│╱
        # y ← ——— ⊙ z     
        #        ╱ ╲
        #       ╱   ╲
        #   (↺)4    3(↻)  
        # 
        beta = deg2rad(self.param.uav.beta_deg)
        l = self.param.uav.l
        Jm = self.param.motor.Jm
        G3 = np.array([
            [Jm*ome[1], -Jm*ome[1], Jm*ome[1], -Jm*ome[1]],
            [-Jm*ome[0], Jm*ome[0], -Jm*ome[0], Jm*ome[0]],
            [0, 0, 0, 0]
        ]) 
        tau = np.zeros(3)
        tau[0] = l * np.sin(beta) * np.array([1.0, -1.0, -1.0, 1.0]) @ motor_thrust
        tau[1] = l * np.cos(beta) * np.array([-1.0, -1.0, 1.0, 1.0]) @ motor_thrust
        tau[2] = np.array([motorcm[0], -motorcm[1], motorcm[2], -motorcm[3]]) @ motor_vel_rad_2
        tau += G3 @ motor_vel_rad
        T_and_tau = np.array([T, tau[0], tau[1], tau[2]])
        
        # sun: 机体阻力在 B 系按轴向速度计算；kh 项近似水平来流造成的附加竖直气动力。
        va_B = v_B - vw_B;
        Fa_B = np.array([ 
            -self.param.aero.kdx * va_B[0], 
            -self.param.aero.kdy * va_B[1], 
            -self.param.aero.kdz * va_B[2] + self.param.aero.kh * (va_B[0] * va_B[0] + va_B[1] * va_B[1])
        ])
        # sun: MuJoCo 的 xfrc_applied 需要世界系力和力矩，因此离开本函数前用 Rbi 旋转。
        force_B = np.array([0,0,T_and_tau[0]]) + Fa_B
        moment_B = T_and_tau[1:4]
        # moment_B[2] = 0
        force = self.state.Rbi @ force_B
        moment = self.state.Rbi @ moment_B
        
        self.quad_debug.total_force_I = force
        self.quad_debug.total_moment_I = moment
        
        # print("T_and_tau:",T_and_tau, flush=True)
        # print("force:",force, flush=True)
        # print("moment:",moment, flush=True)
        # print("T_and_tau:", T_and_tau)
        # 打印周期
        # if time.time() - self.start_time > 0.1:
            
        #     # print("motorct:", motorct, flush=True)
        #     # print("motorcm:", motorcm, flush=True)
        #     # print("motor_thrust:", motor_thrust, flush=True)
        #     # print("motor_vel_rad:", motor_vel_rad, flush=True)
        #     self.start_time = time.time()
        
        return force, moment
    
    def get_actuator_state(self):
        return self.actuator_state
        
    def step(self, dt):
        # now_time = time.time()
        # dt = now_time - self.last_time
        # self.last_time = now_time

        # sun: solve_ivp 仅积分四个电机转速状态，桨角用积分后的转速做显式更新以驱动视觉旋转。
        sol = solve_ivp(self.ode_func, (0, dt), self.actuator_internal_state, method='RK45')
        self.actuator_internal_state = sol.y[:,-1]
        
        # 更新状态
        self.actuator_state.motor_vel_rad = self.actuator_internal_state[0:4]
        self.update_actuator_internal_state()
        self.actuator_state.motor_pos_rad += self.actuator_state.motor_vel_rad * dt
        # 打印周期
        # if self.now_time - self.start_time > 0.05:
        #     self.actuator_internal_state
        #     motor_vel_rad = self.actuator_internal_state[0:4]
        #     motorTm = self.param.motor.Tm_a * rad2rpm(motor_vel_rad) + self.param.motor.Tm_b
        #     print("motor_vel_rad:",motor_vel_rad,flush=True)
        #     print("motorTm:",motorTm,flush=True)
        #     print("Tm_a:",self.param.motor.Tm_a,flush=True)
        #     print("Tm_b:",self.param.motor.Tm_b,flush=True)
        #     self.start_time = time.time()
            
    
    def ode_func(self, t, y):
        # sun: 电机采用转速相关时间常数 Tm(ω)，状态方程为 ω_dot=(ω_cmd-ω)/Tm。
        dxdt = np.zeros(4)
        cur_state = self.ActuatorState()
        
        # 提取状态
        cur_state.motor_vel_rad = y[0:4]
        # 电机转速微分
        motorTm = self.param.motor.Tm_a * rad2rpm(cur_state.motor_vel_rad) + self.param.motor.Tm_b
        motor_rad_dot = (self.input.motor_vel_rad - cur_state.motor_vel_rad) / motorTm
        
        dxdt[0:4] = motor_rad_dot
        # if time.time() - self.start_time > 0.1:
        #     print("motor_vel_rad:",cur_state.motor_vel_rad,flush=True)
        #     print("motor_vel_rpm:",rad2rpm(cur_state.motor_vel_rad),flush=True)
        #     print("motorTm:",motorTm,flush=True)
        #     self.start_time = time.time()
        
        return dxdt
    
    
    
