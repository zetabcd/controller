#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

// sun: Parameter_t 是控制节点的集中参数快照。嵌套结构与 params.yaml 分组对应，
// sun: 便于一次性打印、复制以及在控制周期内无额外 ROS 参数查询地读取。
#include <rclcpp/rclcpp.hpp>

class PX4ControlNode;

struct Parameter_t
{
    struct Gain_t
    {
        // sun: 控制增益依次覆盖位置 P、速度 PID、姿态四元数 P 和角速度 PID 四层环路。
        double gain_pos_x, gain_pos_y, gain_pos_z;
        double gain_vel_p_x, gain_vel_p_y, gain_vel_p_z;
        double gain_vel_i_x, gain_vel_i_y, gain_vel_i_z;
        double gain_vel_d_x, gain_vel_d_y, gain_vel_d_z;
        double gain_quat_x, gain_quat_y, gain_quat_z;
        double gain_rate_p_x, gain_rate_p_y, gain_rate_p_z;
        double gain_rate_i_x, gain_rate_i_y, gain_rate_i_z;
        double gain_rate_d_x, gain_rate_d_y, gain_rate_d_z;
        friend std::ostream& operator<<(std::ostream& os, const Gain_t& gain);
    };
    struct Aero_t
    {
        // sun: rho 为空气密度，kd* 为机体系阻力系数，kh 为水平速度引起的附加竖直气动力系数。
        double rho;
        double kdx, kdy, kdz, kh;
        friend std::ostream& operator<<(std::ostream& os, const Aero_t& drag);
    };
    struct Uav_t
    {
        // sun: Jv* 为机体系主惯量，l/rp/beta_deg 分别为机臂、桨半径和电机安装角。
        double mass;
        double Jvx, Jvy, Jvz;
        double l, rp, beta_deg;
        friend std::ostream& operator<<(std::ostream& os, const Uav_t& uav);
    };
    struct Motor_t
    {
        // sun: Cq*/Ct* 描述随前进比变化的桨系数，u_min/u_max 为单电机推力边界。
        double cq0, ct0;
        double Cq_a, Cq_b, Cq_c;
        double Ct_a, Ct_b, Ct_c;
        double u_min, u_max;
        double rc2speed_a, rc2speed_b, rc2speed_c;
		double hover_percentage;
        friend std::ostream& operator<<(std::ostream& os, const Motor_t& motor);
    };
    struct Filter_t
    {
        // sun: 截止频率单位为 Hz，实际滤波器采样频率由控制周期参数给出。
        double lpf_acc_x_cutoff_hz, lpf_acc_y_cutoff_hz, lpf_acc_z_cutoff_hz;
        double lpf_gyro_x_cutoff_hz, lpf_gyro_y_cutoff_hz, lpf_gyro_z_cutoff_hz;
        friend std::ostream& operator<<(std::ostream& os, const Filter_t& motor);
    };
    struct RCReverse_t
	{
        // sun: 通道反向开关用于适配不同遥控器映射，不应通过修改控制律符号来实现。
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
        friend std::ostream& operator<<(std::ostream& os, const RCReverse_t& rc_rev);
	};
    struct Tuning_t
    {
        // sun: 调参模式可绕过外层参考，直接注入欧拉角或角速度阶跃用于辨识内环。
        bool attitude_loop;
        double euler_des_x_deg, euler_des_y_deg, euler_des_z_deg;
        bool angular_rate_loop;
        double rate_des_x_deg, rate_des_y_deg, rate_des_z_deg;
        friend std::ostream& operator<<(std::ostream& os, const Tuning_t& tuning);
    };
    struct Other_t
    {
        // 最大水平速度，对应PX4的 MPC_XY_VEL_MAX
        double lim_vel_horizontal;  
        // 最大上升速度, 对应PX4的 MPC_Z_VEL_MAX_UP
        double lim_vel_up;
        // 最大下降速度，对应PX4的 MPC_Z_VEL_MAX_DN
        double lim_vel_down;
        // 水平推力余量，对应PX4的 MPC_THR_XY_MARG，用于在垂直推力饱和时进行水平控制
        double lim_thr_xy_margin;
        // 角速度积分器极限，对应PX4的 MC_PR_INT_LIM,MC_RR_INT_LIM,MC_YR_INT_LIM，但是需要进行反归一化
        double lim_rollrate_int, lim_pitchrate_int, lim_yawrate_int; 
        friend std::ostream& operator<<(std::ostream& os, const Other_t& tuning);
    };
    
    Gain_t gain;
    Aero_t aero;
    Uav_t uav;
    Motor_t motor;
    Filter_t filter;
    RCReverse_t rc_reverse;
    Tuning_t tuning;
    Other_t other;
    
    double ctrl_freq_max;
    double ratectrl_freq_max;
    // sun: gra 取重力加速度绝对值，ENU 重力向量在控制器中写作 (0, 0, -gra)。
    double gra;

    friend std::ostream& operator<<(std::ostream& os, const Parameter_t& param);
    Parameter_t& operator=(const Parameter_t& other);
};


#endif
