#ifndef __MOTOR_CALCULATE_H
#define __MOTOR_CALCULATE_H

// sun: 本文件实现电机转速到单桨推力/反扭矩的准稳态映射。四个电机使用 Eigen Array
// sun: 并行计算；速度单位为 rad/s，推力单位为 N，力矩单位为 N·m。
#include <cmath>
#include <cstddef>
#include <iostream>
#include <uav_utils/other_utils.h>
#include <px4ctrl/px4ctrlparam.h>

using namespace uav_utils;

Eigen::Array4d inline get_cts_from_speed(const Eigen::Array4d& motor_rad,const double &vaz_B, const Parameter_t& param)
{
	// sun: 前进比 J = πV/(ωR)；极小量避免零转速除零，clip 限制模型外推到非物理区域。
	Eigen::Array4d Js = clip(pi * vaz_B / (motor_rad * param.uav.rp + 1e-8), 0.0, 1e10);
	Eigen::Array4d CTs = clip(param.motor.Ct_a * Js * Js + param.motor.Ct_b * Js + param.motor.Ct_c, param.motor.Ct_c/10, param.motor.Ct_c);
	Eigen::Array4d cts = CTs / (pi*pi/(4 * param.aero.rho * pow(param.uav.rp,4)));
	return cts;
}
Eigen::Array4d inline get_cms_from_speed(const Eigen::Array4d& motor_rad,const double &vaz_B, const Parameter_t& param)
{
	// sun: 与推力系数相同，先拟合无量纲反扭矩系数，再换算为 Q = cm·ω² 的有量纲系数。
	Eigen::Array4d Js = clip(pi * vaz_B / (motor_rad * param.uav.rp + 1e-8), 0.0, 1e10);
	Eigen::Array4d CMs = clip(param.motor.Cq_a * Js * Js + param.motor.Cq_b * Js + param.motor.Cq_c, param.motor.Cq_c/10, param.motor.Cq_c);
	Eigen::Array4d cms = CMs / (pi*pi/(8 * param.aero.rho * pow(param.uav.rp,5)));
	return cms;
}
Eigen::Array4d inline get_thrust_from_speed(
	const Eigen::Array4d& motor_rad, 
	const Eigen::Array4d& motor_rad_for_J, 
	const Eigen::Vector3d& va_B, 
	const Parameter_t& param)
{
	// sun: motor_rad_for_J 可与输出转速分开，允许使用滞后后的实际转速估计来流效应。
	double vaz_B = va_B[2];
	Eigen::Array4d cts = get_cts_from_speed(motor_rad_for_J, vaz_B, param);
	Eigen::Array4d motor_thrust = motor_rad * motor_rad * cts;
	return motor_thrust;
}
Eigen::Array4d inline get_moment_from_speed(
	const Eigen::Array4d& motor_rad, 
	const Eigen::Array4d& motor_rad_for_J, 
	const Eigen::Vector3d& va_B, 
	const Parameter_t& param)
{
	// sun: 仅机体系轴向来流参与当前桨模型，横向来流的影响由机体气动力模型处理。
	double vaz_B = va_B[2];
	Eigen::Array4d cms = get_cms_from_speed(motor_rad_for_J, vaz_B, param);
	Eigen::Array4d motor_moment = motor_rad * motor_rad * cms;
	return motor_moment;
}
// Eigen::Vector3d get_tau_from_thrust(
// 	const Eigen::Array4d& motor_thrust, 
// 	const Eigen::Array4d& motor_moment, 
// 	const Parameter_t& param)
// {
//     double l = param.uav.l;
//     double beta = deg2rad(param.uav.beta_deg);
//     Eigen::Vector3d tau;
//     tau[0] = l * sin(beta) * Eigen::Vector4d(1.0, -1.0, -1.0, 1.0).dot(motor_thrust.matrix());
//     tau[1] = l * cos(beta) * Eigen::Vector4d(-1.0, -1.0, 1.0, 1.0).dot(motor_thrust.matrix());
//     tau[2] = motor_moment[0] - motor_moment[1] + motor_moment[2] - motor_moment[3];
//     return tau;
// }
#endif
