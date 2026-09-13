#ifndef __CONTROLLER_H
#define __CONTROLLER_H

// sun: 本文件定义顶层飞行控制器的数据契约。控制链路为：
// sun: 参考状态 Ref_State_t + PX4 反馈状态 -> QuadControl -> 姿态/角速度/总推力设定值。
// sun: 除特别说明外，位置、速度和加速度均使用 ROS ENU 惯性坐标系。

#include <rclcpp/time.hpp>
#include <Eigen/Dense>
#include <queue>
#include <cmath>
#include <osqp/osqp.h>
#include <cfloat>

#include <px4debug_msgs/msg/px4ctrl_debug.hpp>
#include <px4ctrl/px4ctrlparam.h>
#include <px4ctrl/input.h>
#include <px4ctrl/frame_transforms.h>
#include <fms_utils/openfsm.h>
#include <uav_utils/filter_utils.h>

// #define USE_OSQP     // 决定是否使用OSQP求解器
// #define USE_NORMAL      // 决定是否使用归一化推力（推荐）
// #define CTRL_MY      // 自定义控制算法

class PX4ControlNode;

using namespace frame_transforms::utils::quaternion;

struct Recorded_State_t
{
	// sun: 记录进入某个飞行状态瞬间的状态，用作悬停、降落等轨迹的初始条件。
	rclcpp::Time time;
	Eigen::Vector3d p;  //位置
	Eigen::Vector3d v;  //速度  
	Eigen::Vector3d a;  //加速度
	Eigen::Quaterniond q;
	double yaw;

	Recorded_State_t(){};

	void set_zero()
	{
		p.setZero();
		v.setZero();
		a.setZero();
		q.setIdentity();
		yaw = 0.0;
	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Ref_State_t
{
	// sun: 平动参考量按 p-v-a-jerk-snap 逐阶排列，便于同时支持反馈控制和微分平坦前馈。
	Eigen::Vector3d p;  //位置
	Eigen::Vector3d v;  //速度  
	Eigen::Vector3d a;  //加速度
	Eigen::Vector3d j;	//jerk
	Eigen::Vector3d s;	//snap
	Eigen::Quaterniond q;
	double yaw_rate;
	double yaw_accel;
	double throttle;

	bool flag_valid_p;
	bool flag_valid_v;
	bool flag_valid_a;

	// sun: 控制器可根据状态机模式选择手动直通或闭环位置控制。
	state fsm_state;

	Ref_State_t()
		: p(Eigen::Vector3d::Zero()),  
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  s(Eigen::Vector3d::Zero()),
		  q(Eigen::Quaterniond::Identity()),
		  yaw_rate(0.0),
		  throttle(-1.0),
		  flag_valid_p(true), flag_valid_v(true), flag_valid_a(true),
		  fsm_state(0){};

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Control_Setpoint_t
{
	// sun: q/bodyrates/thrust 是分层控制器的输出；rate_dot_ref 供角加速度前馈使用。
	Eigen::Quaterniond q;
	Eigen::Vector3d bodyrates; // [rad/s]
	double thrust;	// N
	Eigen::Vector3d rate_dot_ref;

	Eigen::Array4d thro_setpoint;
	Control_Setpoint_t() : 
		bodyrates(Eigen::Vector3d::Zero()),
		thrust(-1.0),
		thro_setpoint(Eigen::Array4d::Constant(-1.0)){};

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class QuadControl
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    QuadControl(PX4ControlNode &);
    px4debug_msgs::msg::Px4ctrlDebug calculateControl(
        const Ref_State_t &des,
        const LocalPose_Data_t &pose,
        const Attitude_Data_t &att,
        const Sensor_Data_t &sens,
		const double &dt,
        Control_Setpoint_t &control_sp,
        const Parameter_t &param
    );
	// sun: 状态切换或重新进入闭环控制时清空积分量和滤波器历史，避免旧状态造成瞬态冲击。
	void resetControlParams();
	// sun: 将牛顿制总推力转换为飞控内部使用的推力信号，并维护在线推力映射模型。
	double computeDesiredCollectiveThrustSignal(const double &thrust_des, const double &uav_mass);
	bool estimateThrustModel(const Eigen::Vector3d &est_a);
	void resetThrustMapping(const Parameter_t &param);
	void init_filters(const Parameter_t &param);
	void reset_filters();
	
private:
    enum procedure_id_ {
        FSM_STATE(manual_on),
        FSM_STATE(manual), 
        FSM_STATE(auto_hover),
        FSM_STATE(cmd),
        FSM_STATE(safe),
        FSM_STATE(err),
    };
    PX4ControlNode& px4controlnode_;
    px4debug_msgs::msg::Px4ctrlDebug debug_msg_;
	std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;
	// sun: 低于该阈值的推力样本激励不足，不适合用于在线辨识推力—加速度比例。
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

	// sun: 速度环积分项位于惯性系，配合推力限幅执行抗积分饱和。
	Eigen::Vector3d vel_int_;

	// sun: 角速度环积分项及正、负方向饱和标志用于底层控制分配的抗饱和逻辑。
	Eigen::Vector3d ome_int_;
	Eigen::Matrix<bool, 3, 1> saturation_positive_, saturation_negative_;
	
	
	// sun: RLS 在线估计参数；thr2acc_ 表示单位推力信号对应的竖直加速度。
	const double rho2_ = 0.998; // do not change
	double thr2acc_;
	double P_;

	// sun: 三轴加速度分别滤波，降低速度环微分反馈对 IMU 高频噪声的放大。
	std::unique_ptr<SecondOrderButterworthLPF> lpf_acc_x_;
	std::unique_ptr<SecondOrderButterworthLPF> lpf_acc_y_;
	std::unique_ptr<SecondOrderButterworthLPF> lpf_acc_z_;

};


#endif
