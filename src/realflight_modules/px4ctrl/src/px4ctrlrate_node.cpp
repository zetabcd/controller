#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <cmath>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <Eigen/Dense>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4debug_msgs/msg/px4ratectrl_debug.hpp>
#include <px4debug_msgs/msg/px4ctrl_debug.hpp>
#include <ratectrl_msgs/msg/rates_thrust_setpoint.hpp>
#include <px4ctrl/input.h>
#include <px4ctrl/px4ctrlparam.h>
#include <px4ctrl/motor_calculate.h>
#include <px4ctrl/motor_feedback.h>
#include <px4ctrl/sliding_window_tvr.h>

#include <fms_utils/openfsm.h>
#include <uav_utils/other_utils.h>
#include <uav_utils/filter_utils.h>
#include "std_srvs/srv/empty.hpp"

// sun: 该节点是高频底层控制器：接收外环的期望角速度/总推力，完成角速度 PID、
// sun: 刚体动力学补偿、四电机控制分配，并向 PX4 或仿真器发布电机归一化指令。

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace uav_utils;

#define RATE 120

class PX4ControlRateNode : public rclcpp::Node
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	PX4ControlRateNode(std::string name, SlidingWindowTVDerivative::swTVR_params_t swtvr_params) 
    : Node(name), sw_tvr_solver_x(swtvr_params), sw_tvr_solver_y(swtvr_params), sw_tvr_solver_z(swtvr_params)
	{
		// sun: 构造时仅建立通信和初始化状态；跨节点参数在握手成功后统一拉取。
		state_timeout_s_ = declare_parameter<double>("diagnostics.state_timeout_s", 0.25);
		calibration_voltage_ = declare_parameter<double>("diagnostics.calibration_voltage", 16.0);
		if (!std::isfinite(state_timeout_s_) || state_timeout_s_ <= 0 ||
			!std::isfinite(calibration_voltage_) || calibration_voltage_ <= 0) {
			throw std::invalid_argument("diagnostic timeout and calibration voltage must be positive");
		}
		feedback_.configure(declare_parameter<double>("diagnostics.esc_timeout_s", 0.25),
			declare_parameter<std::vector<int64_t>>("diagnostics.esc_slots", std::vector<int64_t>{}),
			declare_parameter<std::vector<int64_t>>("diagnostics.motor_functions", {101, 102, 103, 104}));
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;	// Qos设置表
		qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
		qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
		// 发布者
		// Bounded best-effort diagnostics avoid reliable DDS backpressure in the control loop.
		const auto debug_qos = rclcpp::QoS(100).best_effort();
		px4ratectrldebug_publisher_ = this->create_publisher<px4debug_msgs::msg::Px4ratectrlDebug>("/debugPx4/ratectrl", debug_qos);
		motor_feedback_publisher_ = create_publisher<px4debug_msgs::msg::MotorFeedbackDebug>(
			"/debugPx4/motor_feedback", debug_qos);
		esc_status_subscription_ = create_subscription<px4_msgs::msg::EscStatus>(
			declare_parameter<std::string>("diagnostics.esc_topic", "/fmu/out/esc_status"),
			rclcpp::SensorDataQoS(), [this](px4_msgs::msg::EscStatus::ConstSharedPtr msg) {
				feedback_.update(*msg, steadySeconds());
			});
		actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", qos);
		// 订阅者
		sensor_combined_subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(//当前角速度
									"/fmu/out/sensor_combined", 
									qos,
									std::bind(&PX4ControlRateNode::SensorDataCallback, this, std::placeholders::_1));		
		vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(//当前惯性系速度
								   "/fmu/out/vehicle_local_position", 
								   qos,
								   std::bind(&PX4ControlRateNode::LocalPoseDataCallback, this, std::placeholders::_1));
        vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(//当前姿态
								   "/fmu/out/vehicle_attitude", 
								   qos,
								   std::bind(&PX4ControlRateNode::AttitudeCallback, this, std::placeholders::_1));
        rates_thrust_setpoint_subscription_ = this->create_subscription<ratectrl_msgs::msg::RatesThrustSetpoint>(
									"/rates_thrust_setpoint", 
									qos,  
									std::bind(&PX4ControlRateNode::RatesThrustSetpointCallback, this, std::placeholders::_1));
		px4ctrldebug_subscription_ = this->create_subscription<px4debug_msgs::msg::Px4ctrlDebug>(
									"/debugPx4/ctrl", 
									1,
									std::bind(&PX4ControlRateNode::PX4ctrlDebugCallback, this, std::placeholders::_1));		
		is_take_off_ = false;		
        start_time_ = this->get_clock()->now();	

		ome_int_ = Eigen::Vector3d::Zero();
		saturation_positive_ = {false, false, false};
		saturation_negative_ = {false, false, false};
		motor_rad_sol_last_ = Eigen::Vector4d::Zero();
	}

	void reset_start_time()
	{
		start_time_ = this->get_clock()->now();
	}

	void node_handshake_check(const std::string &server_node_name, const std::string &client_node_name)
	{
		// 底层节点只被动等待顶层节点的握手请求。两个节点同时主动
		// 请求对方时，顶层可能在自己的 future 完成后立即转去等待
		// quadsim，不再 spin 底层的反向请求，使底层永久卡住。
		handshake_server_ = this->create_service<std_srvs::srv::Empty>(
			"/"+server_node_name+"/handshake",
			std::bind(&PX4ControlRateNode::handshake_callback, this, std::placeholders::_1, std::placeholders::_2));
		RCLCPP_INFO(this->get_logger(), "\033[33m节点%s：握手服务端已启动，等待节点%s确认...\033[0m",
			server_node_name.c_str(), client_node_name.c_str());
		rclcpp::WallRate wait_rate(100.0);
		while (rclcpp::ok() && !handshake_received_) {
			rclcpp::spin_some(this->get_node_base_interface());
			wait_rate.sleep();
		}
		if (handshake_received_) {
			RCLCPP_INFO(this->get_logger(),
				"\033[32m节点%s：收到节点%s握手确认！开始执行核心逻辑...\033[0m",
				server_node_name.c_str(), client_node_name.c_str());
		}
	}

	void config_from_ros_handle()
	{
		// sun: 参数权威源位于 /px4ctrl_node，底层同步读取以保证质量、惯量和电机模型一致。
		auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this,"/px4ctrl_node");
		// 等待参数服务器启动
		while (!parameter_client->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "Waiting for the parameter service to start...");
		}

		// 从px4ctrl节点获取参数
		param.ratectrl_freq_max = parameter_client->get_parameters({"ratectrl_freq_max"}).front().as_double();

		param.gain.gain_rate_p_x = parameter_client->get_parameters({"gain.gain_rate_p_x"}).front().as_double();
		param.gain.gain_rate_p_y = parameter_client->get_parameters({"gain.gain_rate_p_y"}).front().as_double();
		param.gain.gain_rate_p_z = parameter_client->get_parameters({"gain.gain_rate_p_z"}).front().as_double();
		param.gain.gain_rate_i_x = parameter_client->get_parameters({"gain.gain_rate_i_x"}).front().as_double();
		param.gain.gain_rate_i_y = parameter_client->get_parameters({"gain.gain_rate_i_y"}).front().as_double();
		param.gain.gain_rate_i_z = parameter_client->get_parameters({"gain.gain_rate_i_z"}).front().as_double();
		param.gain.gain_rate_d_x = parameter_client->get_parameters({"gain.gain_rate_d_x"}).front().as_double();
		param.gain.gain_rate_d_y = parameter_client->get_parameters({"gain.gain_rate_d_y"}).front().as_double();
		param.gain.gain_rate_d_z = parameter_client->get_parameters({"gain.gain_rate_d_z"}).front().as_double();

		param.gra = parameter_client->get_parameters({"gra"}).front().as_double();
		param.uav.mass = parameter_client->get_parameters({"uav.mass"}).front().as_double();
		param.uav.Jvx = parameter_client->get_parameters({"uav.Jvx"}).front().as_double();
		param.uav.Jvy = parameter_client->get_parameters({"uav.Jvy"}).front().as_double();
		param.uav.Jvz = parameter_client->get_parameters({"uav.Jvz"}).front().as_double();
		param.uav.l = parameter_client->get_parameters({"uav.l"}).front().as_double();
		param.uav.rp = parameter_client->get_parameters({"uav.rp"}).front().as_double();
		param.uav.beta_deg = parameter_client->get_parameters({"uav.beta_deg"}).front().as_double();

		param.motor.cq0 = parameter_client->get_parameters({"motor.cq0"}).front().as_double();
		param.motor.ct0 = parameter_client->get_parameters({"motor.ct0"}).front().as_double();
		param.motor.Cq_a = parameter_client->get_parameters({"motor.Cq_a"}).front().as_double();
		param.motor.Cq_b = parameter_client->get_parameters({"motor.Cq_b"}).front().as_double();
		param.motor.Cq_c = parameter_client->get_parameters({"motor.Cq_c"}).front().as_double();
		param.motor.Ct_a = parameter_client->get_parameters({"motor.Ct_a"}).front().as_double();
		param.motor.Ct_b = parameter_client->get_parameters({"motor.Ct_b"}).front().as_double();
		param.motor.Ct_c = parameter_client->get_parameters({"motor.Ct_c"}).front().as_double();
		param.motor.u_max = parameter_client->get_parameters({"motor.u_max"}).front().as_double();
		param.motor.u_min = parameter_client->get_parameters({"motor.u_min"}).front().as_double();
		param.motor.rc2speed_a = parameter_client->get_parameters({"motor.rc2speed_a"}).front().as_double();
		param.motor.rc2speed_b = parameter_client->get_parameters({"motor.rc2speed_b"}).front().as_double();
		param.motor.rc2speed_c = parameter_client->get_parameters({"motor.rc2speed_c"}).front().as_double();

		param.filter.lpf_gyro_x_cutoff_hz = parameter_client->get_parameters({"filter.lpf_gyro_x_cutoff_hz"}).front().as_double();
		param.filter.lpf_gyro_y_cutoff_hz = parameter_client->get_parameters({"filter.lpf_gyro_y_cutoff_hz"}).front().as_double();
		param.filter.lpf_gyro_z_cutoff_hz = parameter_client->get_parameters({"filter.lpf_gyro_z_cutoff_hz"}).front().as_double();

		param.aero.rho = parameter_client->get_parameters({"aero.rho"}).front().as_double();
		param.aero.kdx = parameter_client->get_parameters({"aero.kdx"}).front().as_double();
		param.aero.kdy = parameter_client->get_parameters({"aero.kdy"}).front().as_double();
		param.aero.kdz = parameter_client->get_parameters({"aero.kdz"}).front().as_double();
		param.aero.kh = parameter_client->get_parameters({"aero.kh"}).front().as_double();

		param.other.lim_rollrate_int = parameter_client->get_parameters({"other.lim_rollrate_int"}).front().as_double();
		param.other.lim_pitchrate_int = parameter_client->get_parameters({"other.lim_pitchrate_int"}).front().as_double();
		param.other.lim_yawrate_int = parameter_client->get_parameters({"other.lim_yawrate_int"}).front().as_double();

		// std::cout << "param:" << param << std::endl;
	}

	void PX4ctrlDebugCallback(const px4debug_msgs::msg::Px4ctrlDebug::UniquePtr msg)
	{
		input_stamps_[4].update(msg->timestamp, steadySeconds());
		// sun: 根据顶层 FSM 的状态边沿维护起飞标志，不用单独推断油门或高度。
		// uint64_t timestamp;
		// timestamp = msg->timestamp;

		state_data_.fsm_state = msg->state;
		if (state_data_.fsm_state_last == FSM_STATE(manual_on) && state_data_.fsm_state == FSM_STATE(manual))
		{
			is_take_off_ = true;
		}
		if (state_data_.fsm_state_last != FSM_STATE(manual_on) && state_data_.fsm_state == FSM_STATE(manual_on))
		{
			is_take_off_ = false;
		}
		state_data_.fsm_state_last = state_data_.fsm_state;
		// std::cout << "is_take_off_:" << is_take_off_ << std::endl;
	}

	void SensorDataCallback(const px4_msgs::msg::SensorCombined::UniquePtr msg)
	{
		// uint64_t timestamp;
		// timestamp = msg->timestamp;
		// sun: 与顶层输入层保持相同的 FRD -> FLU 符号转换，确保角速度误差在同一坐标系。
		state_data_.sens_w << msg->gyro_rad[0], -msg->gyro_rad[1], -msg->gyro_rad[2];
		input_stamps_[0].update(msg->timestamp, steadySeconds(), state_data_.sens_w.allFinite());
		gyro_integral_dt_us_ = msg->gyro_integral_dt;
		gyro_clipping_ = msg->gyro_clipping;
		gyro_calibration_count_ = msg->gyro_calibration_count;
	}

	void LocalPoseDataCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
	{
		// sun: 扣除 PX4 发布的速度增量修正后再转换到 ENU，供来流速度和桨系数计算使用。
		state_data_.v_I << msg->vx - msg->delta_vxy[0], -(msg->vy - msg->delta_vxy[1]), -(msg->vz - msg->delta_vz);
		input_stamps_[1].update(msg->timestamp, steadySeconds(),
			msg->v_xy_valid && msg->v_z_valid && state_data_.v_I.allFinite());
	}
	void AttitudeCallback(const px4_msgs::msg::VehicleAttitude::UniquePtr msg)
	{
		Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    	state_data_.q = Eigen::Quaterniond(q_ned.w(),q_ned.x(),-q_ned.y(),-q_ned.z());
		state_data_.Rbi = state_data_.q.toRotationMatrix();
		input_stamps_[2].update(msg->timestamp, steadySeconds(),
			state_data_.q.coeffs().allFinite() && std::abs(state_data_.q.norm() - 1.0) < 0.01);
	}

	void RatesThrustSetpointCallback(const ratectrl_msgs::msg::RatesThrustSetpoint::UniquePtr msg)
	{
		// sun: 回调只更新最近一次设定值；高频循环始终使用最新快照，避免控制计算阻塞订阅。
		// uint64_t timestamp;
		// timestamp = msg->timestamp;
		desired_data_.rate_des << msg->bodyrates[0], msg->bodyrates[1], msg->bodyrates[2];
		if(std::isnan(desired_data_.rate_des[0]))
		{
			std::cout << "desired_data_.rate_des contains NAN!" << std::endl;
		}
		desired_data_.thrust_des = msg->thrust;
		desired_data_.rate_dot_ref << msg->rate_dot_ref[0], msg->rate_dot_ref[1], msg->rate_dot_ref[2];
		input_stamps_[3].update(msg->timestamp, steadySeconds(),
			desired_data_.rate_des.allFinite() && desired_data_.rate_dot_ref.allFinite() &&
			std::isfinite(desired_data_.thrust_des));
	}

    void publish_actuator_motors_(Eigen::Array4d motor_thro)
    {
        // sun: PX4 消息固定容纳 12 个执行器，本四旋翼只填前四项，其余保持零。
        std::array<float, 12UL> motor_thrust_all = {0.0f};
        for (size_t i = 0; i < 4; ++i) {
            motor_thrust_all[i] = motor_thro[i];
        }
        px4_msgs::msg::ActuatorMotors msg{};
        msg.control = motor_thrust_all;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        actuator_motors_publisher_->publish(msg);
        debug_msg_.actuator_timestamp = msg.timestamp;
        for (size_t i = 0; i < 4; ++i) debug_msg_.actuator_control[i] = msg.control[i];
        // std::cout << "ctrl_output_.thro_setpoint:" << ctrl_output_.thro_setpoint.transpose() << std::endl;
    }

	void calculateControl(Eigen::Array4d &thro_setpoint)
	{
		const auto calculation_started = std::chrono::steady_clock::now();
		debug_msg_.control_updated = true;
		// sun: dt 使用实际时钟差而非名义频率，使积分项在调度抖动下仍按真实时间累计。
		auto now_time = this->get_clock()->now();
		static auto time_last = now_time;
		double t = (now_time-start_time_).seconds();
		double dt = (now_time-time_last).seconds();
		time_last = now_time;  
		debug_msg_.control_start_timestamp = now_time.nanoseconds() / 1000;
		debug_msg_.elapsed_s = t;
		debug_msg_.dt_s = dt;
		// sun: 当前风速设为零，地速旋转到机体系后近似作为电机轴向来流速度。
		Eigen::Vector3d w_I = Eigen::Vector3d::Zero();
		Eigen::Vector3d va_I = state_data_.v_I - w_I; 
		Eigen::Vector3d va_B = state_data_.Rbi.transpose() * va_I;

		Eigen::Vector3d rate_cur = state_data_.sens_w;
		Eigen::Matrix3d gain_rate_p = Eigen::Vector3d(param.gain.gain_rate_p_x, param.gain.gain_rate_p_y, param.gain.gain_rate_p_z).asDiagonal();
		Eigen::Matrix3d gain_rate_i = Eigen::Vector3d(param.gain.gain_rate_i_x, param.gain.gain_rate_i_y, param.gain.gain_rate_i_z).asDiagonal();
		Eigen::Matrix3d gain_rate_d = Eigen::Vector3d(param.gain.gain_rate_d_x, param.gain.gain_rate_d_y, param.gain.gain_rate_d_z).asDiagonal();

		// sun: 低通角速度用于监视；控制 D 项使用滑窗 TV 正则微分估计的角加速度，
		// sun: 相比直接差分能显著降低陀螺噪声放大。
		Eigen::Vector3d rate_cur_lpf;
		rate_cur_lpf << 
			lpf_gyro_x_->filter(rate_cur[0]),
			lpf_gyro_y_->filter(rate_cur[1]),
			lpf_gyro_z_->filter(rate_cur[2]);
		// 当前角加速度估计（使用滑动窗口二阶TVR求解器）
		Eigen::Vector3d rate_dot_cur;
		SlidingWindowTVDerivative * solvers[] = {&sw_tvr_solver_x, &sw_tvr_solver_y, &sw_tvr_solver_z};
		for (size_t i = 0; i < 3; ++i) {
			const auto started = std::chrono::steady_clock::now();
			rate_dot_cur[i] = solvers[i]->update(t, rate_cur[i]);
			debug_msg_.tvr_time_ms[i] = std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - started).count();
			const auto & d = solvers[i]->diagnostics();
			debug_msg_.tvr_status[i] = d.status;
			debug_msg_.tvr_samples[i] = d.samples;
			debug_msg_.tvr_output_valid[i] = d.output_valid;
			debug_msg_.tvr_mean_dt_s[i] = d.mean_dt;
		}
		// std::cout << "ome_dot_cur:" << ome_dot_cur.transpose() << std::endl;
		// sun: PID 先生成期望角加速度，再通过 Euler 刚体方程
		// sun: τ = J·ω_dot + ω×(Jω) 换算为期望机体系力矩。
		Eigen::Vector3d lim_rate_int(
			param.other.lim_rollrate_int, 
			param.other.lim_pitchrate_int, 
			param.other.lim_yawrate_int);
		Eigen::Matrix3d Jv = Eigen::Vector3d(param.uav.Jvx, param.uav.Jvy, param.uav.Jvz).asDiagonal();
		Eigen::Vector3d rate_err = desired_data_.rate_des - rate_cur;
		copyVector(debug_msg_.rate_err, rate_err);
		copyVector(debug_msg_.i_term, ome_int_);
		copyVector(debug_msg_.saturation_positive_used, saturation_positive_);
		copyVector(debug_msg_.saturation_negative_used, saturation_negative_);
		Eigen::Vector3d ome_dot_des = gain_rate_p * rate_err + ome_int_ + gain_rate_d * (Eigen::Vector3d::Zero() - rate_dot_cur) + desired_data_.rate_dot_ref;
		Eigen::Vector3d tau_des = Jv * ome_dot_des + rate_cur.cross(Jv * rate_cur);
		for (size_t i = 0; i < 3; i++) {
			// sun: 已饱和方向禁止误差继续推高积分量，并在大角速度误差时平滑衰减积分增益。
			if (saturation_positive_[i]) {
				rate_err[i] = std::min(rate_err[i], 0.0);
			}
			if (saturation_negative_[i]) {
				rate_err[i] = std::max(rate_err[i], 0.0);
			}
			double i_factor = rate_err(i) / deg2rad(400.0);
			i_factor = std::max(0.0, 1.0 - i_factor * i_factor);//积分器更新变量
			double rate_i = ome_int_[i] + i_factor * gain_rate_i(i,i) * rate_err(i) * dt;
			debug_msg_.i_factor[i] = i_factor;
			debug_msg_.i_candidate[i] = rate_i;
			debug_msg_.i_update_finite[i] = std::isfinite(rate_i);
			debug_msg_.i_clipped[i] = std::isfinite(rate_i) && std::abs(rate_i) > lim_rate_int[i];
			if (std::isfinite(rate_i)) {
				ome_int_[i] = clip(rate_i, -lim_rate_int[i], lim_rate_int[i]);
			}
		}
		double collective_thrust_des = desired_data_.thrust_des;
		Eigen::Vector4d Ttau_des(collective_thrust_des,tau_des[0],tau_des[1],tau_des[2]);


		// sun: 分配矩阵每周期用当前前进比下的 ct/cm 更新，兼顾来流对推力和反扭矩的影响。
		double l = param.uav.l;
		double beta = deg2rad(param.uav.beta_deg);

		Eigen::Array4d cts = get_cts_from_speed(motor_rad_sol_last_, va_B[2], param);
		Eigen::Array4d cms = get_cms_from_speed(motor_rad_sol_last_, va_B[2], param);
		copyVector(debug_msg_.motor_rad_for_model, motor_rad_sol_last_);
		Eigen::Matrix4d effectiveness;
		// sun: 第一行为总推力，二至四行依次为滚转、俯仰、偏航力矩；列对应 1~4 号电机。
		//          x
		//    (↻)1  ↑  2(↺)
		//        ╲β| ╱ 
		//         ╲│╱
		//  y ← ——— ⊙ z     
		//         ╱ ╲
		//        ╱   ╲
		//    (↺)4    3(↻) 
		effectiveness <<    1.0, 1.0, 1.0, 1.0,
							l*sin(beta), -l*sin(beta), -l*sin(beta), l*sin(beta),
							-l*cos(beta), -l*cos(beta), l*cos(beta), l*cos(beta),
							cms[0]/cts[0], -cms[1]/cts[1], cms[2]/cts[2], -cms[3]/cts[3];
		Eigen::Matrix4d mix = effectiveness.inverse();
		// 将所有小元素设为 0，以避免出现问题
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (abs(mix(i, j)) < 1e-3) {
					mix(i, j) = 0.;
				}
			}
		}

		// sun: 逆混控得到单电机推力后按物理边界裁剪，裁剪后的实际可实现力矩用于抗饱和。
		Eigen::Array4d motor_thrust_sol = (mix * Ttau_des).array();
		copyVector(debug_msg_.motor_thrust_raw, motor_thrust_sol);
		motor_thrust_sol = clip(motor_thrust_sol, param.motor.u_min, param.motor.u_max);

		// sun: 比较期望与可实现力矩，记录每个轴的正/负饱和方向供下一周期冻结对应积分。
		Eigen::Array4d Ttau_sol = effectiveness * motor_thrust_sol.matrix();
		Eigen::Vector3d tau_sol = Ttau_sol.tail<3>();
		saturation_positive_.setConstant(false);
		saturation_negative_.setConstant(false);
		for (size_t i = 0; i < 3; i++)
		{
			if (tau_des[i] - tau_sol[i] > 0.0){
				saturation_positive_[i] = true;
			}
			else if (tau_des[i] - tau_sol[i] < 0.0) {
				saturation_negative_[i] = true;
			}
		}
		
		// sun: 先由 T=ct·ω² 求转速，再反解转速标定二次式得到 [0,1] 油门百分比。
		for (int i = 0; i < 4; i++)
		{
			double thro_setpoint_i = 0.0;
			double motor_rad_sol = std::sqrt(motor_thrust_sol[i] / cts[i]);
			debug_msg_.motor_rad_sol[i] = motor_rad_sol;
			debug_msg_.thro_discriminant[i] = param.motor.rc2speed_b * param.motor.rc2speed_b -
				4 * param.motor.rc2speed_a * (param.motor.rc2speed_c - motor_rad_sol);
			
			if(motor_rad_sol >= (param.motor.rc2speed_c - param.motor.rc2speed_b*param.motor.rc2speed_b/4/param.motor.rc2speed_a)){
				thro_setpoint_i = 1.0;
				debug_msg_.speed_curve_limited[i] = true;
			}else{
				thro_setpoint_i = (-param.motor.rc2speed_b+std::sqrt(param.motor.rc2speed_b*param.motor.rc2speed_b-4*param.motor.rc2speed_a*(param.motor.rc2speed_c-motor_rad_sol)))/2/param.motor.rc2speed_a;
			}
			thro_setpoint[i] = std::min(thro_setpoint_i,1.0);
			debug_msg_.thro_setpoint_i[i] = thro_setpoint_i;
			debug_msg_.thro_setpoint[i] = thro_setpoint[i];
			debug_msg_.motor_reaction_torque_sol[i] = cms[i] * motor_rad_sol * motor_rad_sol;
			debug_msg_.thrust_clipped_low[i] = debug_msg_.motor_thrust_raw[i] < param.motor.u_min;
			debug_msg_.thrust_clipped_high[i] = debug_msg_.motor_thrust_raw[i] > param.motor.u_max;
			motor_rad_sol_last_[i] = motor_rad_sol;
			// 调试消息字段单位为 RPM，控制计算内部仍保持 rad/s。
			debug_msg_.des_motor_rpm[static_cast<std::size_t>(i)] =
				motor_rad_sol * 60.0 / (2.0 * pi);
		}

		debug_msg_.des_rate_dot_x = ome_dot_des[0];
		debug_msg_.des_rate_dot_y = -ome_dot_des[1];
		debug_msg_.des_rate_dot_z = -ome_dot_des[2];

		debug_msg_.cur_rate_dot_x = rate_dot_cur[0];
		debug_msg_.cur_rate_dot_y = -rate_dot_cur[1];
		debug_msg_.cur_rate_dot_z = -rate_dot_cur[2];

		debug_msg_.des_tau_x = tau_des[0];
		debug_msg_.des_tau_y = -tau_des[1];
		debug_msg_.des_tau_z = -tau_des[2];
		// 限幅后分配器真正能够实现的力矩，用于事后判断电机饱和影响。
		debug_msg_.cur_tau_x = tau_sol[0];
		debug_msg_.cur_tau_y = -tau_sol[1];
		debug_msg_.cur_tau_z = -tau_sol[2];

		debug_msg_.des_u_1 = motor_thrust_sol[0];
		debug_msg_.des_u_2 = motor_thrust_sol[1];
		debug_msg_.des_u_3 = motor_thrust_sol[2];
		debug_msg_.des_u_4 = motor_thrust_sol[3];

		copyVector(debug_msg_.rate_cur, rate_cur);
		copyVector(debug_msg_.rate_cur_lpf, rate_cur_lpf);
		copyVector(debug_msg_.rate_des, desired_data_.rate_des);
		copyVector(debug_msg_.rate_err_integrator, rate_err);
		copyVector(debug_msg_.rate_dot_ref, desired_data_.rate_dot_ref);
		copyVector(debug_msg_.rate_dot_cur, rate_dot_cur);
		copyVector(debug_msg_.ome_dot_des, ome_dot_des);
		copyVector(debug_msg_.ome_int_after, ome_int_);
		copyVector(debug_msg_.p_term, (gain_rate_p * (desired_data_.rate_des - rate_cur)).eval());
		copyVector(debug_msg_.d_term, (-gain_rate_d * rate_dot_cur).eval());
		copyVector(debug_msg_.gain_p, gain_rate_p.diagonal());
		copyVector(debug_msg_.gain_i, gain_rate_i.diagonal());
		copyVector(debug_msg_.gain_d, gain_rate_d.diagonal());
		copyVector(debug_msg_.integral_limit, lim_rate_int);
		copyVector(debug_msg_.inertia_diagonal, Jv.diagonal());
		copyVector(debug_msg_.gyro_torque, rate_cur.cross(Jv * rate_cur));
		copyVector(debug_msg_.tau_from_rate_dot, (Jv * rate_dot_cur + rate_cur.cross(Jv * rate_cur)).eval());
		copyVector(debug_msg_.tau_des, tau_des);
		copyVector(debug_msg_.tau_sol, tau_sol);
		copyVector(debug_msg_.tau_residual, (tau_des - tau_sol).eval());
		copyVector(debug_msg_.saturation_positive, saturation_positive_);
		copyVector(debug_msg_.saturation_negative, saturation_negative_);
		copyVector(debug_msg_.velocity_world, state_data_.v_I);
		copyVector(debug_msg_.wind_world, w_I);
		copyVector(debug_msg_.va_b, va_B);
		debug_msg_.attitude_wxyz = {state_data_.q.w(), state_data_.q.x(), state_data_.q.y(), state_data_.q.z()};
		copyVector(debug_msg_.cts, cts);
		copyVector(debug_msg_.cms, cms);
		copyVector(debug_msg_.motor_thrust_sol, motor_thrust_sol);
		for (size_t i = 0; i < 4; ++i) for (size_t j = 0; j < 4; ++j) {
			debug_msg_.effectiveness[4*i+j] = effectiveness(i,j);
			debug_msg_.mix[4*i+j] = mix(i,j);
		}
		debug_msg_.effectiveness_determinant = effectiveness.determinant();
		debug_msg_.thrust_des = collective_thrust_des;
		debug_msg_.thrust_sol = Ttau_sol[0];
		debug_msg_.thrust_residual = collective_thrust_des - Ttau_sol[0];
		debug_msg_.thrust_min = param.motor.u_min;
		debug_msg_.thrust_max = param.motor.u_max;
		debug_msg_.rc2speed_coefficients = {param.motor.rc2speed_a, param.motor.rc2speed_b, param.motor.rc2speed_c};
		debug_msg_.calibration_voltage = calibration_voltage_;
		debug_msg_.ct_coefficients = {param.motor.Ct_a, param.motor.Ct_b, param.motor.Ct_c};
		debug_msg_.cq_coefficients = {param.motor.Cq_a, param.motor.Cq_b, param.motor.Cq_c};
		debug_msg_.air_density = param.aero.rho;
		debug_msg_.propeller_radius = param.uav.rp;
		debug_msg_.arm_length = param.uav.l;
		debug_msg_.arm_angle_rad = beta;
		debug_msg_.nominal_rate_hz = param.ratectrl_freq_max;
		debug_msg_.gyro_lpf_cutoff_hz = {param.filter.lpf_gyro_x_cutoff_hz,
			param.filter.lpf_gyro_y_cutoff_hz, param.filter.lpf_gyro_z_cutoff_hz};
		const auto & tvp = sw_tvr_solver_x.parameters();
		debug_msg_.tvr_parameters = {double(tvp.window_size), tvp.lambda_tv, double(tvp.expend_n),
			double(tvp.n_for_expoly), tvp.atten, double(tvp.order), tvp.weight_scale};
		debug_msg_.control_result_finite = ome_dot_des.allFinite() && tau_des.allFinite() &&
			tau_sol.allFinite() && motor_thrust_sol.allFinite() && thro_setpoint.isFinite().all();
		debug_msg_.solve_time_ms = static_cast<float>(
			std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - calculation_started).count());

		// std::cout << "Td_:" << Td_ << std::endl;
	}
	// A diagnostic sample is finalized only AFTER actuator fallback and publication.
	void runControlCycle()
	{
		const auto started = std::chrono::steady_clock::now();
		debug_msg_ = px4debug_msgs::msg::Px4ratectrlDebug{};
		debug_msg_.previous_cycle_time_ms = previous_cycle_time_ms_;
		const double cycle_start = std::chrono::duration<double>(started - steady_start_).count();
		debug_msg_.cycle_interval_s = cycle_start - previous_cycle_start_s_;
		previous_cycle_start_s_ = cycle_start;
		debug_msg_.cycle_id = ++cycle_id_;
		if (GetThrustDes() >= 0.0) {
			Eigen::Array4d command;
			calculateControl(command);
			for (size_t i = 0; i < 4; ++i) {
				if (std::isnan(command[i])) {
					command[i] = last_command_[i];
					debug_msg_.actuator_fallback[i] = true;
				}
			}
			publish_actuator_motors_(command);
			last_command_ = command;
		} else {
			publish_actuator_motors_(Eigen::Array4d::Constant(-1.0));
		}
		const double now = steadySeconds();
		debug_msg_.timestamp = get_clock()->now().nanoseconds() / 1000;
		debug_msg_.steady_elapsed_us = now * 1e6;
		debug_msg_.fsm_state = state_data_.fsm_state;
		debug_msg_.is_take_off = is_take_off_;
		debug_msg_.gyro_integral_dt_us = gyro_integral_dt_us_;
		debug_msg_.gyro_clipping = gyro_clipping_;
		debug_msg_.gyro_calibration_count = gyro_calibration_count_;
		debug_msg_.control_inputs_valid = true;
		for (size_t i = 0; i < input_stamps_.size(); ++i) {
			debug_msg_.input_timestamp[i] = input_stamps_[i].timestamp;
			debug_msg_.input_sequence[i] = input_stamps_[i].sequence;
			debug_msg_.input_age_s[i] = input_stamps_[i].age(now);
			debug_msg_.input_valid[i] = input_stamps_[i].fresh(now, state_timeout_s_);
			if (i < 4) debug_msg_.control_inputs_valid &= debug_msg_.input_valid[i];
		}
		const Eigen::Vector3d va_b = state_data_.Rbi.transpose() * state_data_.v_I;
		auto feedback = feedback_.sample(now, param, va_b,
			debug_msg_.input_valid[1] && debug_msg_.input_valid[2]);
		feedback.timestamp = debug_msg_.timestamp;
		feedback.cycle_id = debug_msg_.cycle_id;
		feedback.actuator_timestamp = debug_msg_.actuator_timestamp;
		feedback.steady_elapsed_us = debug_msg_.steady_elapsed_us;
		feedback.actuator_control = debug_msg_.actuator_control;
		feedback.calibration_voltage = calibration_voltage_;
		feedback.position_timestamp = input_stamps_[1].timestamp;
		feedback.attitude_timestamp = input_stamps_[2].timestamp;
		feedback.position_age_s = input_stamps_[1].age(now);
		feedback.attitude_age_s = input_stamps_[2].age(now);
		feedback.state_timeout_s = state_timeout_s_;
		if (debug_msg_.control_updated && debug_msg_.control_result_finite) {
			for (size_t i = 0; i < 4; ++i) {
				feedback.desired_motor_rpm[i] = debug_msg_.motor_rad_sol[i] * 60.0 / (2*pi);
				if (feedback.rpm_valid[i]) feedback.rpm_error[i] =
					feedback.motor_rpm[i] - feedback.desired_motor_rpm[i];
			}
		}
		debug_msg_.work_time_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - started).count();
		debug_msg_.deadline_missed = debug_msg_.work_time_ms > 1000.0 / param.ratectrl_freq_max;
		px4ratectrldebug_publisher_->publish(debug_msg_);
		motor_feedback_publisher_->publish(feedback);
		previous_cycle_time_ms_ = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - started).count();
	}
	double GetThrustDes()
	{
		return desired_data_.thrust_des;
	}
	void init_filters(const Parameter_t &param)
	{
		double fs = param.ratectrl_freq_max;
		lpf_gyro_x_ = std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_gyro_x_cutoff_hz, fs);
		lpf_gyro_y_ = std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_gyro_y_cutoff_hz, fs);
		lpf_gyro_z_ =  std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_gyro_z_cutoff_hz, fs);
	}
	void reset_filters()
	{
		lpf_gyro_x_->reset();
		lpf_gyro_y_->reset();
		lpf_gyro_z_->reset();
	}

	Parameter_t param;
	bool has_new_message = false;

private:
    template<typename T, size_t N, typename Vector>
    static void copyVector(std::array<T, N> & destination, const Vector & source)
    {
        for (size_t i = 0; i < N; ++i) destination[i] = source[i];
    }
    double steadySeconds() const
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - steady_start_).count();
    }
    const std::chrono::steady_clock::time_point steady_start_{std::chrono::steady_clock::now()};
    std::array<rate_diagnostics::InputStamp, 5> input_stamps_;
    uint64_t gyro_integral_dt_us_{0};
    uint8_t gyro_clipping_{0};
    uint8_t gyro_calibration_count_{0};
    double previous_cycle_time_ms_{rate_diagnostics::nan};
    double previous_cycle_start_s_{rate_diagnostics::nan};
    uint64_t cycle_id_{0};
    double state_timeout_s_{0.25};
    double calibration_voltage_{16.0};
    rate_diagnostics::MotorFeedback feedback_;
    // Initial NaN used to fall back to itself. Start with the existing inactive command.
    Eigen::Array4d last_command_{Eigen::Array4d::Constant(-1.0)};
    rclcpp::Publisher<px4debug_msgs::msg::MotorFeedbackDebug>::SharedPtr motor_feedback_publisher_;
    rclcpp::Subscription<px4_msgs::msg::EscStatus>::SharedPtr esc_status_subscription_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr handshake_server_;
	bool handshake_received_{false};
    rclcpp::Time start_time_;
    enum procedure_id_ {
        FSM_STATE(manual_on),
        FSM_STATE(manual), 
        FSM_STATE(auto_hover),
        FSM_STATE(cmd),
        FSM_STATE(safe),
        FSM_STATE(err),
    };
    struct State_Data_t{
		Eigen::Vector3d sens_w{Eigen::Vector3d::Constant(rate_diagnostics::nan)};
		Eigen::Vector3d v_I{Eigen::Vector3d::Constant(rate_diagnostics::nan)};
		Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
		Eigen::Matrix3d Rbi{Eigen::Matrix3d::Identity()};
		state fsm_state{0};
		state fsm_state_last{0};
	};
    struct Desired_Data_t{
		Eigen::Vector3d rate_des{Eigen::Vector3d::Constant(rate_diagnostics::nan)};
		double thrust_des = -1.0; // [N]
		Eigen::Vector3d rate_dot_ref{Eigen::Vector3d::Constant(rate_diagnostics::nan)};
	};
    State_Data_t state_data_;
	Desired_Data_t desired_data_;
	bool is_take_off_;
	rclcpp::TimerBase::SharedPtr timer_;
	px4debug_msgs::msg::Px4ratectrlDebug debug_msg_;

	// sun: 三个独立 TVR 求解器分别估计滚转、俯仰、偏航角加速度。
    SlidingWindowTVDerivative sw_tvr_solver_x,sw_tvr_solver_y,sw_tvr_solver_z;

	// 角速度环
	Eigen::Vector3d ome_int_;
	Eigen::Matrix<bool, 3, 1> saturation_positive_, saturation_negative_;
	Eigen::Array4d motor_rad_sol_last_;

	// 滤波器
	std::unique_ptr<SecondOrderButterworthLPF> lpf_gyro_x_;
	std::unique_ptr<SecondOrderButterworthLPF> lpf_gyro_y_;
	std::unique_ptr<SecondOrderButterworthLPF> lpf_gyro_z_;

	// 发布者
    rclcpp::Publisher<px4debug_msgs::msg::Px4ratectrlDebug>::SharedPtr px4ratectrldebug_publisher_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
	// 订阅者
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_subscription_;
    rclcpp::Subscription<ratectrl_msgs::msg::RatesThrustSetpoint>::SharedPtr rates_thrust_setpoint_subscription_;
	rclcpp::Subscription<px4debug_msgs::msg::Px4ctrlDebug>::SharedPtr px4ctrldebug_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;

	// 握手服务端的回调（仅需返回成功，无需处理数据）
    void handshake_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response){
        (void)response; // 消除未使用警告
		handshake_received_ = true;
    }
};



int main(int argc, char *argv[])
{
	// sun: 主循环以 ratectrl_freq_max 运行；无有效总推力时发布 -1，表示执行器停用。
	// 设置标准输出（stdout）为无缓冲模式
	// 让所有对标准输出的操作立即生效，而不是等到缓冲区满或遇到换行符时才输出。这在需要即时输出信息的情况下（如实时日志记录、调试信息输出等）非常有用。
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);

	SlidingWindowTVDerivative::swTVR_params_t sw_params = {//然后求解一个“既接近原始差分，又不要变化太剧烈”的角加速度序列u
        // sun: 200 点窗口在平滑性与延迟之间折中，后续参数控制 TV 正则和边缘外推。
        200,    // window_size
        0.9,    // lambda_tv控制平滑程度
        100,    // expend_n
        25,     // n_for_expoly
        9,      // atten
        1,      // order
        2       // weight_scale
    };
	auto node = std::make_shared<PX4ControlRateNode>("px4ctrlrate_node", sw_params);
	node->node_handshake_check("px4ctrlrate_node","px4ctrl_node");
	RCLCPP_INFO(node->get_logger(), "\033[33m节点px4ctrlrate_node：节点px4ctrl_node已上线，控制开始...\033[0m");

	/* 读取参数 */
	node->config_from_ros_handle();
	node->publish_actuator_motors_(Eigen::Array4d(-1.0,-1.0,-1.0,-1.0));

	// 初始化滤波器
	node->init_filters(node->param);

	rclcpp::WallRate loop_rate(node->param.ratectrl_freq_max);
	node->reset_start_time();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->runControlCycle();
        loop_rate.sleep();
    }


	rclcpp::shutdown();
	return 0;
}
