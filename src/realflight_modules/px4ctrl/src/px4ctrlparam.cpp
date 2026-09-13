#include <ostream>
#include <px4ctrl/px4ctrlparam.h>
#include <px4ctrl/px4ctrl_node.h>
#include <uav_utils/other_utils.h>

// sun: 本文件提供参数树的可读打印与深复制，主要用于启动诊断和运行时参数快照管理。

using namespace uav_utils;

std::ostream& operator<<(std::ostream& os, const Parameter_t::Gain_t& gain)
{
    os << "  Gain:" << std::endl;
    // os << "    filt_lpf=" << gain.filt_lpf << ", filt_hpf=" << gain.filt_hpf << std::endl;
    os << "    gain_pos_x=" << gain.gain_pos_x << ", gain_pos_y=" << gain.gain_pos_y << ", gain_pos_z=" << gain.gain_pos_z << std::endl;
    os << "    gain_vel_p_x=" << gain.gain_vel_p_x << ", gain_vel_p_y=" << gain.gain_vel_p_y << ", gain_vel_p_z=" << gain.gain_vel_p_z << std::endl;
    os << "    gain_vel_i_x=" << gain.gain_vel_i_x << ", gain_vel_i_y=" << gain.gain_vel_i_y << ", gain_vel_i_z=" << gain.gain_vel_i_z << std::endl;
    os << "    gain_vel_d_x=" << gain.gain_vel_d_x << ", gain_vel_d_y=" << gain.gain_vel_d_y << ", gain_vel_d_z=" << gain.gain_vel_d_z << std::endl;
    os << "    gain_quat_x=" << gain.gain_quat_x << ", gain_quat_y=" << gain.gain_quat_y << ", gain_quat_z=" << gain.gain_quat_z << std::endl;
    os << "    gain_rate_p_x=" << gain.gain_rate_p_x << ", gain_rate_p_y=" << gain.gain_rate_p_y << ", gain_rate_p_z=" << gain.gain_rate_p_z << std::endl;
    os << "    gain_rate_i_x=" << gain.gain_rate_i_x << ", gain_rate_i_y=" << gain.gain_rate_i_y << ", gain_rate_i_z=" << gain.gain_rate_i_z << std::endl;
    os << "    gain_rate_d_x=" << gain.gain_rate_d_x << ", gain_rate_d_y=" << gain.gain_rate_d_y << ", gain_rate_d_z=" << gain.gain_rate_d_z << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Parameter_t::Aero_t& aero)
{
    os << "  AERO:" << std::endl;
    os << "    rho=" << aero.rho << std::endl;
    os << "    kdx=" << aero.kdx << ", kdy=" << aero.kdy << ", kdz=" << aero.kdz << ", kh=" << aero.kh << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Parameter_t::Uav_t& uav)
{
    os << "  UAV:" << std::endl;
    os << "    mass=" << uav.mass << std::endl;
    os << "    Jvx=" << uav.Jvx << ", Jvy=" << uav.Jvy << ", Jvz=" << uav.Jvz << std::endl;
    os << "    l=" << uav.l << ", rp=" << uav.rp << ", beta_deg=" << uav.beta_deg << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Parameter_t::Motor_t& motor)
{
    os << "  MOTOR:" << std::endl;
    os << "    cq0=" << motor.cq0 << ", ct0=" << motor.ct0 << std::endl;
    os << "    Cq_a=" << motor.Cq_a << ", Cq_b=" << motor.Cq_b << ", Cq_c=" << motor.Cq_c << std::endl;
    os << "    Ct_a=" << motor.Ct_a << ", Ct_b=" << motor.Ct_b << ", Ct_c=" << motor.Ct_c << std::endl;
    os << "    u_min=" << motor.u_min << ", u_max=" << motor.u_max << std::endl;
    os << "    rc2speed_a=" << motor.rc2speed_a << ", rc2speed_b=" << motor.rc2speed_b << ", rc2speed_c=" << motor.rc2speed_c << std::endl;
    os << "    hover_percentage=" << motor.hover_percentage << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Parameter_t::Filter_t& filter)
{
    os << "  Filter:" << std::endl;
    os << "    lpf_acc_x_cutoff_hz=" << filter.lpf_acc_x_cutoff_hz 
       << ", lpf_acc_y_cutoff_hz=" << filter.lpf_acc_y_cutoff_hz 
       << ", lpf_acc_z_cutoff_hz=" << filter.lpf_acc_z_cutoff_hz << std::endl;
    os << "    lpf_gyro_x_cutoff_hz=" << filter.lpf_gyro_x_cutoff_hz 
       << ", lpf_gyro_y_cutoff_hz=" << filter.lpf_gyro_y_cutoff_hz 
       << ", lpf_gyro_z_cutoff_hz=" << filter.lpf_gyro_z_cutoff_hz << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Parameter_t::RCReverse_t& rc_rev)
{
    os << "  RCReverse:" << std::endl;
    os << "    roll=" << (rc_rev.roll ? "true" : "false") 
       << ", pitch=" << (rc_rev.pitch ? "true" : "false") 
       << ", yaw=" << (rc_rev.yaw ? "true" : "false") 
       << ", throttle=" << (rc_rev.throttle ? "true" : "false") << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Parameter_t::Tuning_t& tuning)
{
    os << "  Tuning:" << std::endl;
    os << "    attitude_loop=" << (tuning.attitude_loop ? "true" : "false") 
       << ", euler_des_x_deg=" << tuning.euler_des_x_deg 
       << ", euler_des_y_deg=" << tuning.euler_des_y_deg 
       << ", euler_des_z_deg=" << tuning.euler_des_z_deg << std::endl;
    os << "    angular_rate_loop=" << (tuning.angular_rate_loop ? "true" : "false") 
       << ", rate_des_x_deg=" << tuning.rate_des_x_deg 
       << ", rate_des_y_deg=" << tuning.rate_des_y_deg 
       << ", rate_des_z_deg=" << tuning.rate_des_z_deg << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Parameter_t::Other_t& other)
{
    os << "  Other:" << std::endl;
    os << "    rollrate_int_lim=" << other.lim_rollrate_int 
       << ", pitchrate_int_lim=" << other.lim_pitchrate_int 
       << ", yawrate_int_lim=" << other.lim_yawrate_int << std::endl;
    return os;
}
// 主结构体的 << 运算符定义
std::ostream& operator<<(std::ostream& os, const Parameter_t& param)
{
    os << "Parameter_t 所有参数：" << std::endl;
    // 基础参数
    os << "  基础参数：" << std::endl;
    os << "    gra=" << param.gra << " (重力加速度)" << std::endl;
    os << "    ctrl_freq_max=" << param.ctrl_freq_max << " (最大控制频率)" << std::endl;
    
    // 嵌套结构体
    os << param.gain << std::endl;
    os << param.aero << std::endl;
    os << param.uav << std::endl;
    os << param.motor << std::endl;
    os << param.filter << std::endl;
    os << param.rc_reverse << std::endl;
    os << param.tuning << std::endl;
    os << param.other << std::endl;

    return os;
}

Parameter_t& Parameter_t::operator=(const Parameter_t& param) 
{
    // sun: 显式列出各分组可避免浅拷贝遗漏语义；新增参数分组时也应同步更新此处。
    if (this != &param) {
        // 复制所有非引用成员
        gain = param.gain;
        aero = param.aero;
        uav = param.uav;
        motor = param.motor;
        filter = param.filter;
        rc_reverse = param.rc_reverse;
        ctrl_freq_max = param.ctrl_freq_max;
        ratectrl_freq_max = param.ratectrl_freq_max;
        gra = param.gra;
        tuning = param.tuning;
        other = param.other;
    }
    return *this;
}
