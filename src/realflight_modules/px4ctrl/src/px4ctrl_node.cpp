#include <px4ctrl/px4ctrl_node.h>
#include <px4ctrl/ommpc.h>
#include <px4ctrl/fcu_reboot.h>
#include <uav_utils/other_utils.h>
#include <uav_utils/project_paths.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>

// sun: 顶层节点只负责通信和调度。订阅回调写入 fsm 的数据缓存，定时循环调用
// sun: PX4CtrlFSM::process()，从而让控制计算与 ROS 消息到达时刻解耦。

using namespace uav_utils;

PX4ControlNode::PX4ControlNode(std::string name) : Node(name), fsm(*this)
{
	// sun: 传感器话题采用 best-effort 和深度 1，优先获取最新样本而不是补处理过期队列。
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;	// Qos设置表
	qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
	qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
	// sun: 外环输出自定义角速度/推力消息，状态机心跳和调试量分别独立发布。
	fsm.offboard_control_mode_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	fsm.px4ctrldebug_publisher = this->create_publisher<px4debug_msgs::msg::Px4ctrlDebug>("/debugPx4/ctrl",1);
	fsm.rates_thrust_setpoint_publisher = this->create_publisher<ratectrl_msgs::msg::RatesThrustSetpoint>("/rates_thrust_setpoint",qos);
	// 订阅者
#ifdef SIMULATION
#ifndef USE_WITHOUT_RC

	
	joystick_subscription_ = this->create_subscription<joy_msgs::msg::JoyStick>(
							 "/joyStick",
							 1,
							 std::bind(&RC_Data_t::feed, &fsm.rc_data, std::placeholders::_1));
#endif
#else
	qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	
	manual_control_setpoint_subscription_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
											"/fmu/out/manual_control_setpoint", 
											qos,
											std::bind(&RC_Data_t::feed, &fsm.rc_data, std::placeholders::_1));
#endif

		/* Topic                               消息类型                                 发布方               接收方                                         用途
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━
   /joyStick                           joy_msgs/msg/JoyStick                    外部摇杆节点         px4ctrl_node                                   仿真遥控器输入
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /fmu/out/manual_control_setpoint    px4_msgs/msg/ManualControlSetpoint       PX4                  px4ctrl_node                                   真机遥控器输入
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /fmu/out/vehicle_status_v1          px4_msgs/msg/VehicleStatus               PX4                  px4ctrl_node                                   解锁、模式、飞行前检查
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /fmu/out/battery_status             px4_msgs/msg/BatteryStatus               PX4                  px4ctrl_node                                   电池状态
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /fmu/out/sensor_combined            px4_msgs/msg/SensorCombined              PX4或quadsim_node    两个控制节点                                   原IMU源（宏=0）
   /fmu/out/vehicle_angular_velocity   px4_msgs/msg/VehicleAngularVelocity      PX4                  两个控制节点                                   已滤波角速度（宏=1）
   /fmu/out/vehicle_acceleration       px4_msgs/msg/VehicleAcceleration         PX4                  px4ctrl_node                                   已滤波比力（宏=1）
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /fmu/out/vehicle_attitude           px4_msgs/msg/VehicleAttitude             PX4或quadsim_node    两个控制节点                                   当前姿态
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /fmu/out/vehicle_local_position     px4_msgs/msg/VehicleLocalPosition        PX4或quadsim_node    两个控制节点                                   位置、速度、有效标志
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /rates_thrust_setpoint              ratectrl_msgs/msg/RatesThrustSetpoint    px4ctrl_node         px4ctrlrate_node                               期望角速度、总推力
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /fmu/in/actuator_motors             px4_msgs/msg/ActuatorMotors              px4ctrlrate_node     PX4或quadsim_node                              四电机控制量
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /fmu/in/offboard_control_mode       px4_msgs/msg/OffboardControlMode         px4ctrl_node         PX4                                            声明Offboard控制层级
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /debugPx4/ctrl                      px4debug_msgs/msg/Px4ctrlDebug           px4ctrl_node         px4ctrlrate_node、quadsim_node、PlotJuggler    外环调试和状态机状态
  ──────────────────────────────────  ───────────────────────────────────────  ───────────────────  ─────────────────────────────────────────────  ────────────────────────
   /debugPx4/ratectrl                  px4debug_msgs/msg/Px4ratectrlDebug       px4ctrlrate_node     调试工具                                       角速度环和电机分配调试 */

	qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
	// sun: 每个订阅回调直接绑定到对应输入缓存的 feed()，避免节点层重复做数据转换。
	vehicle_status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
								   "/fmu/out/vehicle_status_v1",
								   qos,
								   std::bind(&States_Data_t::feed, &fsm.sta_data, std::placeholders::_1));	
	battery_status_position_subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
								   "/fmu/out/battery_status", 
								   qos,
								   std::bind(&Battery_Data_t::feed, &fsm.bat_data, std::placeholders::_1));	
#if PX4CTRL_USE_FILTERED_IMU
	// 独立传感器流使用 volatile/best-effort，兼容 PX4 DDS 实时发布。
	const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
	vehicle_angular_velocity_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
		"/fmu/out/vehicle_angular_velocity", imu_qos,
		std::bind(&Sensor_Data_t::feed_angular_velocity, &fsm.sens_data, std::placeholders::_1));
	vehicle_acceleration_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAcceleration>(
		"/fmu/out/vehicle_acceleration", imu_qos,
		std::bind(&Sensor_Data_t::feed_acceleration, &fsm.sens_data, std::placeholders::_1));
	RCLCPP_INFO(get_logger(), "IMU source: vehicle_angular_velocity + vehicle_acceleration");
#else
	sensor_combined_subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
								   "/fmu/out/sensor_combined", 
								   qos,
								   std::bind(&Sensor_Data_t::feed, &fsm.sens_data, std::placeholders::_1));		
	RCLCPP_INFO(get_logger(), "IMU source: sensor_combined");
#endif
	vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
								   "/fmu/out/vehicle_attitude", 
								   qos,
								   std::bind(&Attitude_Data_t::feed, &fsm.att_data, std::placeholders::_1));								   							
	vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
								   "/fmu/out/vehicle_local_position", 
								   qos,
								   std::bind(&LocalPose_Data_t::feed, &fsm.pose_data, std::placeholders::_1));
	// sun: 实机通过 PX4 VehicleCommand 服务请求模式切换和解锁；仿真编译时跳过等待。
	fsm.vehicle_command_client = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
#ifndef SIMULATION
	/* 等待vehicle_command服务上线 */
	while (!fsm.vehicle_command_client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "\033[31mservice not available, waiting again...\033[0m");
	}
#endif

    main_timer_ = this->create_wall_timer(
      500ms, std::bind(&PX4ControlNode::timer_callback, this));

}

void PX4ControlNode::timer_callback()
{
    // RCLCPP_INFO(this->get_logger(), "Publishing:");
}

void PX4ControlNode::node_handshake_check(const std::string &server_node_name, const std::string &client_node_name)
{
	// sun: 两个控制节点互相暴露空服务；只有双方均能调用对方后才进入控制循环。
	handshake_server_ = this->create_service<std_srvs::srv::Empty>(
		"/"+server_node_name+"/handshake",
		std::bind(&PX4ControlNode::handshake_callback, this, std::placeholders::_1, std::placeholders::_2));
	RCLCPP_INFO(this->get_logger(), "\033[33m节点%s：握手服务端已启动，等待节点%s上线...\033[0m",
		server_node_name.c_str(), client_node_name.c_str());
	handshake_client_ = this->create_client<std_srvs::srv::Empty>("/"+client_node_name+"/handshake");
	while (rclcpp::ok())
	{
		if (handshake_client_->wait_for_service(1s)) // 1秒超时检测
		{
			// 发送握手请求
			auto request = std::make_shared<std_srvs::srv::Empty::Request>();
			auto result = handshake_client_->async_send_request(request);
			
			// 等待响应（确认节点2在线）
			if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
				rclcpp::FutureReturnCode::SUCCESS)
			{
				RCLCPP_INFO(this->get_logger(), "\033[32m节点%s：与节点%s握手成功！开始执行核心逻辑...\033[0m",
					server_node_name.c_str(), client_node_name.c_str());
				break; // 退出等待，执行核心逻辑
			}
		}
		RCLCPP_WARN(this->get_logger(), "\033[33m节点%s：未检测到节点%s，继续等待...\033[0m",
			server_node_name.c_str(), client_node_name.c_str());
	}
}

void PX4ControlNode::config_from_ros_handle()
{
    // sun: ROS 2 参数必须先声明才能读取；这里的默认值只保证类型正确，正常值由 YAML 覆盖。
    this->declare_parameter<double>("ctrl_freq_max",200.0);
    this->declare_parameter<double>("ratectrl_freq_max",400.0);
    this->declare_parameter<double>("gra",9.8);

    this->declare_parameter<double>("uav.mass",1.0);
    this->declare_parameter<double>("uav.Jvx",0.0);
    this->declare_parameter<double>("uav.Jvy",0.0);
    this->declare_parameter<double>("uav.Jvz",0.0);
    this->declare_parameter<double>("uav.l",0.0);
    this->declare_parameter<double>("uav.rp",0.0);
    this->declare_parameter<double>("uav.beta_deg",0.0);

    this->declare_parameter<double>("motor.cq0",0.0);
    this->declare_parameter<double>("motor.Cq_a",0.0);
    this->declare_parameter<double>("motor.Cq_b",0.0);
    this->declare_parameter<double>("motor.Cq_c",0.0);
    this->declare_parameter<double>("motor.ct0",0.0);
    this->declare_parameter<double>("motor.Ct_a",0.0);
    this->declare_parameter<double>("motor.Ct_b",0.0);
    this->declare_parameter<double>("motor.Ct_c",0.0);
    this->declare_parameter<double>("motor.u_max",0.0);
    this->declare_parameter<double>("motor.u_min",0.0);
    this->declare_parameter<double>("motor.rc2speed_a",0.0);
    this->declare_parameter<double>("motor.rc2speed_b",0.0);
    this->declare_parameter<double>("motor.rc2speed_c",0.0);
    this->declare_parameter<double>("motor.hover_percentage",0.0);

	this->declare_parameter<double>("filter.lpf_acc_x_cutoff_hz",0.0);
	this->declare_parameter<double>("filter.lpf_acc_y_cutoff_hz",0.0);
	this->declare_parameter<double>("filter.lpf_acc_z_cutoff_hz",0.0);
	this->declare_parameter<double>("filter.lpf_gyro_x_cutoff_hz",0.0);
	this->declare_parameter<double>("filter.lpf_gyro_y_cutoff_hz",0.0);
	this->declare_parameter<double>("filter.lpf_gyro_z_cutoff_hz",0.0);

    this->declare_parameter<double>("gain.gain_pos_x",0.0);
    this->declare_parameter<double>("gain.gain_pos_y",0.0);
    this->declare_parameter<double>("gain.gain_pos_z",0.0);
    this->declare_parameter<double>("gain.gain_vel_p_x",0.0);
    this->declare_parameter<double>("gain.gain_vel_p_y",0.0);
    this->declare_parameter<double>("gain.gain_vel_p_z",0.0);
	this->declare_parameter<double>("gain.gain_vel_i_x",0.0);
    this->declare_parameter<double>("gain.gain_vel_i_y",0.0);
    this->declare_parameter<double>("gain.gain_vel_i_z",0.0);
    this->declare_parameter<double>("gain.gain_vel_d_x",0.0);
    this->declare_parameter<double>("gain.gain_vel_d_y",0.0);
    this->declare_parameter<double>("gain.gain_vel_d_z",0.0);
    this->declare_parameter<double>("gain.gain_quat_x",0.0);
    this->declare_parameter<double>("gain.gain_quat_y",0.0);
    this->declare_parameter<double>("gain.gain_quat_z",0.0);
    this->declare_parameter<double>("gain.gain_rate_p_x",0.0);
    this->declare_parameter<double>("gain.gain_rate_p_y",0.0);
    this->declare_parameter<double>("gain.gain_rate_p_z",0.0);
    this->declare_parameter<double>("gain.gain_rate_i_x",0.0);
    this->declare_parameter<double>("gain.gain_rate_i_y",0.0);
    this->declare_parameter<double>("gain.gain_rate_i_z",0.0);
	this->declare_parameter<double>("gain.gain_rate_d_x",0.0);
    this->declare_parameter<double>("gain.gain_rate_d_y",0.0);
    this->declare_parameter<double>("gain.gain_rate_d_z",0.0);

    this->declare_parameter<double>("aero.rho",0.0);
    this->declare_parameter<double>("aero.kdx",0.0);
    this->declare_parameter<double>("aero.kdy",0.0);
    this->declare_parameter<double>("aero.kdz",0.0);
    this->declare_parameter<double>("aero.kh",0.0);

    this->declare_parameter<bool>("rc_reverse.roll",true);
    this->declare_parameter<bool>("rc_reverse.pitch",true);
    this->declare_parameter<bool>("rc_reverse.yaw",true);
    this->declare_parameter<bool>("rc_reverse.throttle",true);

    this->declare_parameter<bool>("tuning.attitude_loop",false);
    this->declare_parameter<double>("tuning.euler_des_x_deg",0.0);
    this->declare_parameter<double>("tuning.euler_des_y_deg",0.0);
    this->declare_parameter<double>("tuning.euler_des_z_deg",0.0);
    this->declare_parameter<bool>("tuning.angular_rate_loop",false);
    this->declare_parameter<double>("tuning.rate_des_x_deg",0.0);
    this->declare_parameter<double>("tuning.rate_des_y_deg",0.0);
    this->declare_parameter<double>("tuning.rate_des_z_deg",0.0);

	this->declare_parameter<double>("other.lim_rollrate_int",0.0);
	this->declare_parameter<double>("other.lim_pitchrate_int",0.0);
	this->declare_parameter<double>("other.lim_yawrate_int",0.0);

    // “起飞 + 八字”CMD 轨迹参数。length/width 为包围盒全尺寸，speed 为八字段
    // 最大路径速度；参数只决定参考轨迹，不改变 FSM 状态转换或控制器求解设置。
    this->declare_parameter<double>("trajectory.figure_eight.takeoff_height", 1.0);
    this->declare_parameter<double>("trajectory.figure_eight.takeoff_duration", 2.5);
    this->declare_parameter<double>("trajectory.figure_eight.settle_duration", 0.5);
    this->declare_parameter<double>("trajectory.figure_eight.length", 2.0);
    this->declare_parameter<double>("trajectory.figure_eight.width", 1.2);
    this->declare_parameter<double>("trajectory.figure_eight.speed", 1.0);
    this->declare_parameter<int>("trajectory.figure_eight.laps", 1);
    // 论文最小-jerk QP 多圈翻滚轨迹；只有 PX4CTRL_CMD_TRAJECTORY=3 时使用。
    this->declare_parameter<double>("trajectory.five_turn.takeoff_height", 1.0);
    this->declare_parameter<double>("trajectory.five_turn.takeoff_duration", 2.5);
    this->declare_parameter<double>("trajectory.five_turn.settle_duration", 0.5);
    this->declare_parameter<int>("trajectory.five_turn.revolutions", 5);
    this->declare_parameter<int>("trajectory.five_turn.waypoints_per_revolution", 16);
    this->declare_parameter<double>("trajectory.five_turn.radius", 1.0);
    this->declare_parameter<double>("trajectory.five_turn.axial_speed", 0.5);
    this->declare_parameter<double>("trajectory.five_turn.entry_duration", 1.5);
    this->declare_parameter<double>("trajectory.five_turn.roll_duration", 9.0);
    this->declare_parameter<double>("trajectory.five_turn.exit_duration", 1.5);
    this->declare_parameter<double>(
        "trajectory.five_turn.velocity_tracking_weight", 100.0);
    this->declare_parameter<double>(
        "trajectory.five_turn.acceleration_tracking_weight", 10.0);
    this->declare_parameter<double>(
        "trajectory.five_turn.jerk_tracking_weight", 0.1);
    // 在线控制只读取离线优化器保存的最终状态序列，不声明也不运行任何
    // OmTrajectoryOptimizer 求解参数。
    this->declare_parameter<std::string>("trajectory.omtraj.file", "");
    this->declare_parameter<double>("trajectory.point_to_point.max_speed", 1.0);
    this->declare_parameter<double>("trajectory.point_to_point.max_acceleration", 1.5);

    // MPC/NMPC 的求解参数由 YAML 管理，在 main() 中读取后只配置一次控制器。
    this->declare_parameter<int>("mpc.horizon", 8);
    this->declare_parameter<double>("mpc.prediction_dt", 0.01);
    this->declare_parameter<std::vector<double>>(
        "mpc.body_rate_max", {14.0, 14.0, 14.0});
    this->declare_parameter<std::vector<double>>(
        "mpc.state_weight",
        {15000.0, 15000.0, 15000.0, 40.0, 40.0, 40.0, 80.0, 80.0, 80.0});
    this->declare_parameter<std::vector<double>>(
        "mpc.input_weight", {0.5, 0.6, 0.6, 0.6});
    this->declare_parameter<double>("mpc.terminal_weight_scale", 1.0);

    this->declare_parameter<int>("nmpc.horizon", 12);
    this->declare_parameter<double>("nmpc.prediction_dt", 0.04);
    this->declare_parameter<int>("nmpc.maximum_iterations", 60);
    this->declare_parameter<std::vector<double>>(
        "nmpc.body_rate_max", {14.0, 14.0, 14.0});

    // 获取参数
    this->get_parameter("ctrl_freq_max", param.ctrl_freq_max);
    this->get_parameter("ratectrl_freq_max", param.ratectrl_freq_max);
    this->get_parameter("gra", param.gra);

    this->get_parameter("uav.mass", param.uav.mass);
    this->get_parameter("uav.Jvx", param.uav.Jvx);
    this->get_parameter("uav.Jvy", param.uav.Jvy);
    this->get_parameter("uav.Jvz", param.uav.Jvz);
    this->get_parameter("uav.l", param.uav.l);
    this->get_parameter("uav.rp", param.uav.rp);
    this->get_parameter("uav.beta_deg", param.uav.beta_deg);

    this->get_parameter("motor.Cq_a", param.motor.Cq_a);
    this->get_parameter("motor.Cq_b", param.motor.Cq_b);
    this->get_parameter("motor.Cq_c", param.motor.Cq_c);
    this->get_parameter("motor.Ct_a", param.motor.Ct_a);
    this->get_parameter("motor.Ct_b", param.motor.Ct_b);
    this->get_parameter("motor.Ct_c", param.motor.Ct_c);
    this->get_parameter("motor.rc2speed_a", param.motor.rc2speed_a);
    this->get_parameter("motor.rc2speed_b", param.motor.rc2speed_b);
    this->get_parameter("motor.rc2speed_c", param.motor.rc2speed_c);

	this->get_parameter("filter.lpf_acc_x_cutoff_hz", param.filter.lpf_acc_x_cutoff_hz);
	this->get_parameter("filter.lpf_acc_y_cutoff_hz", param.filter.lpf_acc_y_cutoff_hz);
	this->get_parameter("filter.lpf_acc_z_cutoff_hz", param.filter.lpf_acc_z_cutoff_hz);

    this->get_parameter("gain.gain_pos_x", param.gain.gain_pos_x);
    this->get_parameter("gain.gain_pos_y", param.gain.gain_pos_y);
    this->get_parameter("gain.gain_pos_z", param.gain.gain_pos_z);
    this->get_parameter("gain.gain_vel_p_x", param.gain.gain_vel_p_x);
    this->get_parameter("gain.gain_vel_p_y", param.gain.gain_vel_p_y);
    this->get_parameter("gain.gain_vel_p_z", param.gain.gain_vel_p_z);
    this->get_parameter("gain.gain_vel_i_x", param.gain.gain_vel_i_x);
    this->get_parameter("gain.gain_vel_i_y", param.gain.gain_vel_i_y);
    this->get_parameter("gain.gain_vel_i_z", param.gain.gain_vel_i_z);
    this->get_parameter("gain.gain_vel_d_x", param.gain.gain_vel_d_x);
    this->get_parameter("gain.gain_vel_d_y", param.gain.gain_vel_d_y);
    this->get_parameter("gain.gain_vel_d_z", param.gain.gain_vel_d_z);
    this->get_parameter("gain.gain_quat_x", param.gain.gain_quat_x);
    this->get_parameter("gain.gain_quat_y", param.gain.gain_quat_y);
    this->get_parameter("gain.gain_quat_z", param.gain.gain_quat_z);
    this->get_parameter("gain.gain_rate_p_x", param.gain.gain_rate_p_x);
    this->get_parameter("gain.gain_rate_p_y", param.gain.gain_rate_p_y);
    this->get_parameter("gain.gain_rate_p_z", param.gain.gain_rate_p_z);
    this->get_parameter("gain.gain_rate_i_x", param.gain.gain_rate_i_x);
    this->get_parameter("gain.gain_rate_i_y", param.gain.gain_rate_i_y);
    this->get_parameter("gain.gain_rate_i_z", param.gain.gain_rate_i_z);
	this->get_parameter("gain.gain_rate_d_x", param.gain.gain_rate_d_x);
    this->get_parameter("gain.gain_rate_d_y", param.gain.gain_rate_d_y);
    this->get_parameter("gain.gain_rate_d_z", param.gain.gain_rate_d_z);

    this->get_parameter("aero.rho", param.aero.rho);
    this->get_parameter("aero.kdx", param.aero.kdx);
    this->get_parameter("aero.kdy", param.aero.kdy);
    this->get_parameter("aero.kdz", param.aero.kdz);
    this->get_parameter("aero.kh", param.aero.kh);

    this->get_parameter("rc_reverse.roll", param.rc_reverse.roll);
    this->get_parameter("rc_reverse.pitch", param.rc_reverse.pitch);
    this->get_parameter("rc_reverse.yaw", param.rc_reverse.yaw);
    this->get_parameter("rc_reverse.throttle", param.rc_reverse.throttle);

    this->get_parameter("tuning.attitude_loop", param.tuning.attitude_loop);
    this->get_parameter("tuning.euler_des_x_deg", param.tuning.euler_des_x_deg);
    this->get_parameter("tuning.euler_des_y_deg", param.tuning.euler_des_y_deg);
    this->get_parameter("tuning.euler_des_z_deg", param.tuning.euler_des_z_deg);
    this->get_parameter("tuning.angular_rate_loop", param.tuning.angular_rate_loop);
    this->get_parameter("tuning.rate_des_x_deg", param.tuning.rate_des_x_deg);
    this->get_parameter("tuning.rate_des_y_deg", param.tuning.rate_des_y_deg);
    this->get_parameter("tuning.rate_des_z_deg", param.tuning.rate_des_z_deg);

    param.motor.ct0 = param.motor.Ct_c / (pi * pi / (4 * param.aero.rho * pow(param.uav.rp,4)));
    param.motor.cq0 = param.motor.Cq_c / (pi * pi / (8 * param.aero.rho * pow(param.uav.rp,5)));
    param.motor.hover_percentage = (-param.motor.rc2speed_b+std::sqrt(param.motor.rc2speed_b*param.motor.rc2speed_b-4*param.motor.rc2speed_a*(param.motor.rc2speed_c-std::sqrt((param.uav.mass * param.gra / 4.0) / param.motor.ct0))))/2/param.motor.rc2speed_a;
    param.motor.u_min = param.motor.rc2speed_c * param.motor.rc2speed_c * param.motor.ct0;
    double speed_rad_max = param.motor.rc2speed_a + param.motor.rc2speed_b + param.motor.rc2speed_c;//满油门对应的最大转速
    param.motor.u_max = speed_rad_max * speed_rad_max * param.motor.ct0;//单电机最大推力

    if (param.tuning.attitude_loop || param.tuning.angular_rate_loop)
    {   // 如果处于attitude_loop或angular_rate_loop调参模式，忽略空气动力参数
        param.aero.kdx = 0.0;
        param.aero.kdy = 0.0;
        param.aero.kdz = 0.0;
        param.aero.kh = 0.0;
        this->set_parameter(rclcpp::Parameter("aero.kdx", 0.0));
        this->set_parameter(rclcpp::Parameter("aero.kdy", 0.0));
        this->set_parameter(rclcpp::Parameter("aero.kdz", 0.0));
        this->set_parameter(rclcpp::Parameter("aero.kh", 0.0));
    }

    this->set_parameter(rclcpp::Parameter("motor.u_min", param.motor.u_min));
    this->set_parameter(rclcpp::Parameter("motor.u_max", param.motor.u_max));
    this->set_parameter(rclcpp::Parameter("motor.hover_percentage", param.motor.hover_percentage));
    this->set_parameter(rclcpp::Parameter("motor.ct0", param.motor.ct0));
    this->set_parameter(rclcpp::Parameter("motor.cq0", param.motor.cq0));
	
	param.other.lim_vel_horizontal = 12.; 
	param.other.lim_vel_up = 3.;
	param.other.lim_vel_down = 1.5;
	param.other.lim_thr_xy_margin = 0.3 * 4 * param.motor.u_max;
	
	// 角速度积分限幅计算（根据最大电机推力和力矩的缩放计算，缩放系数参考PX4的 MC_RR_INT_LIM）
	double l = param.uav.l;
	double beta = deg2rad(param.uav.beta_deg);
	Eigen::Array4d motor_thrust_max = param.motor.u_max * Eigen::Array4d::Ones();
	Eigen::Array4d motor_moment_max = param.motor.u_max / param.motor.ct0 * param.motor.cq0 * Eigen::Array4d::Ones();
	Eigen::Vector3d moment_max;
	moment_max[0] = l * sin(beta) * Eigen::Vector4d(1.0, 0.0, 0.0, 1.0).dot(motor_thrust_max.matrix());
	moment_max[1] = l * cos(beta) * Eigen::Vector4d(0.0, 0.0, 1.0, 1.0).dot(motor_thrust_max.matrix());
	moment_max[2] = motor_moment_max[0] + motor_moment_max[2];
	param.other.lim_rollrate_int = moment_max[0]/param.uav.Jvx * 0.3;//力矩除以惯量得到角加速度限值
	param.other.lim_pitchrate_int = moment_max[1]/param.uav.Jvy * 0.3;
	param.other.lim_yawrate_int = moment_max[2]/param.uav.Jvz * 0.3;

	this->set_parameter(rclcpp::Parameter("other.lim_rollrate_int", param.other.lim_rollrate_int));
    this->set_parameter(rclcpp::Parameter("other.lim_pitchrate_int", param.other.lim_pitchrate_int));
    this->set_parameter(rclcpp::Parameter("other.lim_yawrate_int", param.other.lim_yawrate_int));

	// std::cout << "param:" << param << std::endl;
}
int main(int argc, char *argv[])
{
	// 设置标准输出（stdout）为无缓冲模式
	// 让所有对标准输出的操作立即生效，而不是等到缓冲区满或遇到换行符时才输出。这在需要即时输出信息的情况下（如实时日志记录、调试信息输出等）非常有用。
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	auto node = std::make_shared<PX4ControlNode>("px4ctrl_node");
	
	/* 读取参数 */
	node->config_from_ros_handle();
	node->param_init = node->param;
#if PX4CTRL_CMD_TRAJECTORY == 4
	const auto trajectory_file = node->get_parameter("trajectory.omtraj.file").as_string();
	node->set_parameter(rclcpp::Parameter(
		"trajectory.omtraj.file",
		uav_utils::projectPath(
			trajectory_file.empty() ? "datalog/omtraj/omtraj_optimized.csv" : trajectory_file,
			ament_index_cpp::get_package_share_directory("px4ctrl")).string()));
#endif

	// ---------------------------------------------------------------------
	// [ACTIVE] OmMpcControl 参数初始化
	// ---------------------------------------------------------------------
	// PX4ControlNode 的成员构造顺序使 FSM 早于 ROS 参数读取，因此必须在
	// config_from_ros_handle() 之后把真实重力、频率和执行器边界写入 MPC。

	//这一段代码是保证mpc控制器的时候参数有效
#if PX4CTRL_USE_OMMPC_PRIMARY_CONTROLLER
	if (!(node->param.uav.mass > 0.0) ||
		node->param.motor.u_max <= node->param.motor.u_min)
	{
		RCLCPP_FATAL(
			node->get_logger(),
			"[px4ctrl] Invalid MPC actuator parameters: mass=%g, motor=[%g,%g] N",
			node->param.uav.mass, node->param.motor.u_min, node->param.motor.u_max);
		rclcpp::shutdown();
		return 1;
	}
#if PX4CTRL_PRIMARY_CONTROLLER == 1
	OmMpcOptions controller_options = node->fsm.controller.options();
	controller_options.horizon = static_cast<int>(
		node->get_parameter("mpc.horizon").as_int());
	controller_options.prediction_dt =
		node->get_parameter("mpc.prediction_dt").as_double();
	const auto body_rate_max =
		node->get_parameter("mpc.body_rate_max").as_double_array();
	const auto state_weight =
		node->get_parameter("mpc.state_weight").as_double_array();
	const auto input_weight =
		node->get_parameter("mpc.input_weight").as_double_array();
	if (body_rate_max.size() == 3) {
		controller_options.body_rate_max =
			Eigen::Map<const Eigen::Vector3d>(body_rate_max.data());
	}
	if (state_weight.size() == 9) {
		controller_options.state_weight =
			Eigen::Map<const Eigen::Matrix<double, 9, 1>>(state_weight.data());
	}
	if (input_weight.size() == 4) {
		controller_options.input_weight =
			Eigen::Map<const Eigen::Matrix<double, 4, 1>>(input_weight.data());
	}
	controller_options.terminal_weight_scale =
		node->get_parameter("mpc.terminal_weight_scale").as_double();
#else
	SolverNmpcOptions controller_options = node->fsm.controller.options();
	controller_options.horizon = static_cast<int>(
		node->get_parameter("nmpc.horizon").as_int());
	controller_options.prediction_dt =
		node->get_parameter("nmpc.prediction_dt").as_double();
	controller_options.maximum_iterations = static_cast<int>(
		node->get_parameter("nmpc.maximum_iterations").as_int());
	const auto body_rate_max =
		node->get_parameter("nmpc.body_rate_max").as_double_array();
	if (body_rate_max.size() == 3) {
		controller_options.body_rate_max =
			Eigen::Map<const Eigen::Vector3d>(body_rate_max.data());
	}
#endif
	controller_options.gravity = node->param.gra;
	controller_options.thrust_acceleration_min =
		4.0 * node->param.motor.u_min / node->param.uav.mass;
	controller_options.thrust_acceleration_max =
		4.0 * node->param.motor.u_max / node->param.uav.mass;
	node->fsm.controller.setOptions(controller_options);
#if PX4CTRL_CMD_TRAJECTORY == 3
	// 稠密最小-jerk QP 在启动阶段完成；此时控制循环尚未运行，约 1 s 的
	// 计算不会中断 PX4 Offboard 心跳。生成失败则禁止带病进入飞行流程。
	if (!node->fsm.prepare_five_turn_cmd_trajectory()) {
		RCLCPP_FATAL(
			node->get_logger(),
			"[px4ctrl] Cannot prepare five-turn CMD trajectory");
		rclcpp::shutdown();
		return 1;
	}
#elif PX4CTRL_CMD_TRAJECTORY == 4
	// 这里只读取离线优化结果并做完整性检查，不运行轨迹优化器。
	if (!node->fsm.load_offline_omtraj_template()) {
		RCLCPP_FATAL(
			node->get_logger(),
			"[px4ctrl] Cannot prepare omtraj CMD trajectory");
		rclcpp::shutdown();
		return 1;
	}
#endif
#endif

	node->node_handshake_check("px4ctrl_node", "px4ctrlrate_node");

#ifdef SIMULATION
	node->node_handshake_check("px4ctrl_node", "quadsim_node");
#endif


	/*
	 * [SIMULATOR STARTUP READINESS BARRIER]
	 * 握手只表示 quadsim_node 在线；它会在握手后继续创建发布器并初始化模型。
	 * USE_WITHOUT_RC 则会在 FSM 启动后仅产生一次 AUTO_HOVER 挡位边沿。如果
	 * 该边沿早于第一帧状态到达，切换会因 No pose 被拒绝且不会自动重试。
	 *
	 * 因此仿真模式必须在启动原 FSM 之前确认：三类消息确实到达、仍然新鲜，
	 * 且局部位置有效。连续多个检查周期满足条件，用于避开发布器刚建立时的瞬态。
	 * 这里只建立节点启动屏障，不修改仿真器、FSM 状态或状态切换条件。
	 */
	double RC_EXE_RATE = 120;
#ifdef SIMULATION
	constexpr int kRequiredReadyCycles = 3;
	int ready_cycles = 0;
	rclcpp::Time last_wait_log = node->get_clock()->now();
	RCLCPP_INFO(node->get_logger(),
		"\033[33m[PX4CTRL] Waiting for valid simulator pose/attitude/imu...\033[0m");
	while (rclcpp::ok())
	{
		rclcpp::WallRate(RC_EXE_RATE).sleep();
		rclcpp::spin_some(node);
		const rclcpp::Time state_time = node->get_clock()->now();
		const bool messages_arrived =
			node->fsm.pose_data.recv_new_msg &&
			node->fsm.att_data.timestamp != 0U &&
			node->fsm.sens_data.timestamp != 0U;
		const bool messages_fresh =
			node->fsm.pose_is_received(state_time) &&
			node->fsm.att_is_received(state_time) &&
			node->fsm.sens_is_received(state_time);
		// LocalPose 的 validity 字段由第一条消息赋值；消息到达前不可读取。
		const bool pose_valid = messages_arrived &&
			node->fsm.pose_is_valid(node->fsm.pose_data);

		ready_cycles = messages_arrived && messages_fresh && pose_valid ?
			ready_cycles + 1 : 0;
		if (ready_cycles >= kRequiredReadyCycles)
		{
			RCLCPP_INFO(node->get_logger(),
				"\033[32m[PX4CTRL] Simulator state ready; starting FSM.\033[0m");
			break;
		}
		if ((state_time - last_wait_log).seconds() >= 1.0)
		{
			RCLCPP_WARN(node->get_logger(),
				"\033[33m[PX4CTRL] Simulator state not ready: arrived=%d, fresh=%d, "
				"pose_valid=%d.\033[0m",
				messages_arrived, messages_fresh, pose_valid);
			last_wait_log = state_time;
		}
	}
#else
	/* [ORIGINAL NON-SIMULATION TIMING] 固定等待 1 s。 */
	const rclcpp::Time input_wait_start = node->get_clock()->now();
    while (rclcpp::ok()) 
	{
		rclcpp::WallRate(RC_EXE_RATE).sleep();
		rclcpp::spin_some(node);
		if ((node->get_clock()->now()-input_wait_start).seconds() > 1)break;
	}
#endif
	
#ifndef USE_WITHOUT_RC
	/* 遥控器信号校验 */
	rclcpp::Time now_time = node->get_clock()->now();
	RCLCPP_INFO(node->get_logger(),"\033[33m[PX4CTRL] Waiting for RC...\033[0m");
    while (rclcpp::ok()) 
	{
	
		rclcpp::WallRate(RC_EXE_RATE).sleep(); //这句话一定要在前面，因为rcv_stamp的初始值和node->now()很相近
		rclcpp::spin_some(node);
		static bool flag_rc_is_received = false;

		// std::cout << node->fsm.rc_data.aux1 << std::endl;
		if (node->fsm.rc_is_received(node->now()))
		{
			if (!flag_rc_is_received) RCLCPP_INFO(node->get_logger(),"\033[32m[PX4CTRL] RC received.\033[0m");
			flag_rc_is_received = true;
			// std::cout << node->fsm.rc_is_downinit() << std::endl;
			/* 判断初始状态切换摇杆是否在最下面 aux1和aux2*/
			if (node->fsm.rc_is_downinit()){
				break;
			}
			else{
				static rclcpp::Time last_time = node->get_clock()->now();
				now_time = node->get_clock()->now();
				// std::cout << (now_time - last_time).seconds() << std::endl;
				if ((now_time - last_time).seconds() > 1)
				{
					RCLCPP_INFO(node->get_logger(),"\033[33m[PX4CTRL] RC initial mode is not DOWN!.\033[0m");
					last_time = node->get_clock()->now();
				}
			}
		}
    }
#endif	
	/* 启动FSM */
	rclcpp::WallRate loop_rate(node->param.ctrl_freq_max);
#if !PX4CTRL_USE_OMMPC_PRIMARY_CONTROLLER
	// [LEGACY QuadControl] 原位置/速度环的加速度滤波器初始化。
    node->fsm.controller.init_filters(node->param);
#endif

	node->fsm.reset_start_time(); // 握手完成后，重置状态机的起始时间
#ifndef SIMULATION
	px4ctrl::FcuRebootClient fcu_reboot(*node);
	uint64_t last_aux5_change_count = node->fsm.rc_data.aux5_change_count;
#endif
	//最后一个while循环，程序会陷在这里面
	while (rclcpp::ok()) 
	{
		rclcpp::spin_some(node);
#ifdef SIMULATION
		node->fsm.process();
#else
		// AUX5 FCU reboot belongs to the main loop, before both kill and arming
		// gates. No dependency on manual_on or on running the control FSM.
		const uint64_t aux5_count = node->fsm.rc_data.aux5_change_count;
		const auto status_age = (node->now() - node->fsm.sta_data.rcv_stamp).seconds();
		const bool reboot_allowed = node->fsm.rc_is_received(node->now()) &&
			node->fsm.sta_data.timestamp != 0 && status_age >= 0.0 && status_age < 1.0 &&
			node->fsm.sta_data.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
		fcu_reboot.update(aux5_count - last_aux5_change_count, reboot_allowed);
		last_aux5_change_count = aux5_count; // Consume changes even while armed.
		//判断是否解锁和紧急开关，
		static bool flag_armed = false;
		static bool flag_kill = false;
		if (!node->fsm.rc_is_kill()){//aux4 up，down为正常，正常进入if
			if (node->fsm.vs_is_armed()){
				if (!flag_armed) {
					RCLCPP_INFO(node->get_logger(),"\033[32m[PX4CTRL] Armed!\033[0m");
					flag_armed = true;
				}
				node->fsm.process();//只有在arm后才能执行fsm
			}else{
				if (flag_armed) {
					RCLCPP_INFO(node->get_logger(),"\033[31m[PX4CTRL] Disarmed!\033[0m");
					flag_armed = false;
				}
			}
			flag_kill = false;
		}else{
			if (!flag_kill) RCLCPP_INFO(node->get_logger(),"\033[31m[PX4CTRL] Kill Switch Engaged!\033[0m");
			flag_kill = true;
		}
#endif
		loop_rate.sleep();//node->param.ctrl_freq_max
    }

	rclcpp::shutdown();
	return 0;
}
void PX4ControlNode::init_param()
{
	// sun: 将参数服务器当前值复制成一个一致的 Parameter_t 快照，供本周期控制算法读取。
	param = param_init;
}
