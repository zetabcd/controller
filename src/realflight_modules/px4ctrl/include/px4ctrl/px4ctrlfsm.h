#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

// sun: PX4CtrlFSM 是控制系统的流程调度层：检查输入新鲜度和有效性、执行飞行模式
// sun: 跳转、生成对应参考量，再调用 QuadControl 并向 PX4 发布角速度/推力设定值。

#include <iostream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <ratectrl_msgs/msg/rates_thrust_setpoint.hpp>
#include <px4debug_msgs/msg/px4ctrl_debug.hpp>

#include <fms_utils/openfsm.h>
#include <px4ctrl/barrel_roll_trajectory.h>
#include <px4ctrl/controller.h>
#include <px4ctrl/figure_eight_trajectory.h>
#include <px4ctrl/ommpc.h>
#include <px4ctrl/input.h>
#include <px4ctrl/minimum_snap_trajectory.h>
#include <px4ctrl/five_turn_trajectory.h>
#include <px4ctrl/solver_nmpc.h>

class PX4ControlNode;

// 顶层控制器唯一切换开关。所有实现都保留，修改一个数字即可切换：
// 0=QuadControl, 1=OmMpcControl, 2=Ipopt+Eigen, 3=acados,
// 4=NLopt+Eigen, 5=Ipopt+CasADi。
#ifndef PX4CTRL_PRIMARY_CONTROLLER
#define PX4CTRL_PRIMARY_CONTROLLER 3
#endif

// 旧条件宏现在表示“使用可接收 OmTrajectoryResult 的轨迹控制器”。保留名称
// 是为了避免扰动已经严格隔离的 FSM 安全/轨迹逻辑；只有 0=QuadControl 为假。
#define PX4CTRL_USE_OMMPC_PRIMARY_CONTROLLER (PX4CTRL_PRIMARY_CONTROLLER != 0)

// MPC 的 CMD 轨迹唯一切换开关：0=minimum-snap，1=barrel roll，
// 2=figure eight，3=论文最小-jerk多圈翻滚，4=本项目 OmTrajectoryOptimizer。
// 危险机动不作为默认项自动启用。
#ifndef PX4CTRL_CMD_TRAJECTORY
#define PX4CTRL_CMD_TRAJECTORY 2
#endif

class PX4CtrlFSM
{
public:
    // sun: 各输入缓存由订阅回调异步更新，process() 在定时器线程中读取其最新快照。
    RC_Data_t rc_data;
    States_Data_t sta_data;
    Battery_Data_t bat_data;
    Sensor_Data_t sens_data;
    Attitude_Data_t att_data;
    LocalPose_Data_t pose_data;
    Recorded_State_t record_state_data;
    rclcpp::Time now_time;
    px4debug_msgs::msg::Px4ctrlDebug debug_msg;
    // ---------------------------------------------------------------------
    // 顶层控制器选择（一次只启用一个）
    // ---------------------------------------------------------------------
    // [LEGACY] 原 QuadControl 的全部实现仍保存在 controller.h/.cpp。
#if PX4CTRL_PRIMARY_CONTROLLER == 1
    // [ACTIVE] 流形 MPC；保持同名 controller 使 FSM 公共调用接口无需改写。
    OmMpcControl controller;
#elif PX4CTRL_PRIMARY_CONTROLLER == 2
    // [ALTERNATIVE] Ipopt + Eigen，直接单重射并使用 L-BFGS Hessian。
    IpoptEigenNmpcControl controller;
#elif PX4CTRL_PRIMARY_CONTROLLER == 3
    // [ALTERNATIVE] acados 生成式 SQP + HPIPM 后端。
    AcadosNmpcControl controller;
#elif PX4CTRL_PRIMARY_CONTROLLER == 4
    // [ALTERNATIVE] NLopt + Eigen，边界约束 L-BFGS 后端。
    NloptEigenNmpcControl controller;
#elif PX4CTRL_PRIMARY_CONTROLLER == 5
    // [ALTERNATIVE] CasADi 自动微分建模 + Ipopt 后端。
    IpoptCasadiNmpcControl controller;
#else
    // [LEGACY ACTIVE] 将上面的宏改为 0 即恢复原串级控制器。
    QuadControl controller;
#endif
    bool service_done;

    PX4CtrlFSM(PX4ControlNode &);
    void process();
    void reset_start_time();
    void set_init_ref();
    void set_hover_ref();
    void set_point_hover(const double &x, const double &y, const double &z);
    void set_manual_ref(const double &dt, bool reset_yaw_des=false);
    void set_manual_postion_ref(const double &dt, bool reset_yaw_des=false);
    void set_2D8_ref();
    void set_land_ref();
    void set_fixed_wing_ref(const Attitude_Data_t &att, const Sensor_Data_t &sens);
    
    void record_position();
    bool rc_is_received(const rclcpp::Time &now_time);
    bool rc_is_downinit();
    bool rc_is_kill();
    bool vs_is_armed();
    bool rc_is_armed();
    bool pose_is_valid(const LocalPose_Data_t &pose);
    bool pose_is_received(const rclcpp::Time &now_time);
    bool att_is_received(const rclcpp::Time &now_time);
    bool bat_is_received(const rclcpp::Time &now_time);
    bool sens_is_received(const rclcpp::Time &now_time);
    bool recv_new_pose();
    bool arm();
    void publish_rates_thrust_setpoint();
#if PX4CTRL_USE_OMMPC_PRIMARY_CONTROLLER
    // 多圈论文轨迹包含一次较大的稠密 QP 求解；必须在控制循环启动前缓存，
    // 进入 CMD 时只做刚体坐标变换，避免中断 Offboard 心跳。
    bool prepare_five_turn_cmd_trajectory();
    // 只读取离线优化器保存的最终状态序列，不在控制进程中求解。
    bool load_offline_omtraj_template();
#endif

    // sun: 这些发布器分别负责外部模式心跳、飞行器命令、控制设定值和调试信息。
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher;
    rclcpp::Publisher<ratectrl_msgs::msg::RatesThrustSetpoint>::SharedPtr rates_thrust_setpoint_publisher;
    rclcpp::Publisher<px4debug_msgs::msg::Px4ctrlDebug>::SharedPtr px4ctrldebug_publisher;
    

    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client;

private:
    // 遥控器边沿只持续一个回调周期，因此将模式切换意图锁存到服务确认返回。
    enum class ModeSwitchTarget {
        NONE,
        OFFBOARD,
        MANUAL,
    };
    // VehicleCommand 服务是异步的；记录请求类型可避免旧响应被新的切模操作误用。
    enum class VehicleCommandRequest {
        NONE,
        OFFBOARD,
        MANUAL,
        ARM,
    };

    // sun: control_sp_ 是本周期控制输出，ref_ 是由当前状态生成的统一参考状态。
    Control_Setpoint_t control_sp_;
    Ref_State_t ref_; 
    rclcpp::Time start_time_;
    rclcpp::Time last_time_;
    double t_, dt_;
    double manual_yaw_;
    // AUTO_HOVER 中摇杆移动目标点；参考按速度/加速度边界连续生成。
    Eigen::Vector3d point_target_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d point_ref_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d point_ref_velocity_{Eigen::Vector3d::Zero()};
    double point_ref_yaw_{0.0};
    double point_max_speed_{1.0};
    double point_max_acceleration_{1.0};
    bool point_ref_initialized_{false};
    // sun: fsm_ 保存当前/上次/下次状态，procedure_list_ 将状态编号映射到成员处理函数。
    FSM fsm_;
    // 状态机跳转列表
    Procedure procedure_list_[6];
    state pre_state_, cur_state_, nxt_state_;
    // Node节点指针
    PX4ControlNode& px4controlnode_;
    // 状态ID ，顺序要求与 procedure_list[]对应 
    enum procedure_id_ {
        FSM_STATE(manual_on),
        FSM_STATE(manual), 
        FSM_STATE(auto_hover),
        FSM_STATE(cmd),
        FSM_STATE(safe),
        FSM_STATE(err),
    };
    // sun: err_code_ 的地址注册给通用 FSM，got_err_ 仅在步进失败时读取错误码。
    int err_code_;
    int *got_err_;
    // 服务响应返回参数
    
    uint8_t service_result_;
    ModeSwitchTarget pending_mode_switch_{ModeSwitchTarget::NONE};
    VehicleCommandRequest active_service_request_{VehicleCommandRequest::NONE};
    VehicleCommandRequest completed_service_request_{VehicleCommandRequest::NONE};
    bool service_request_pending_{false};
    // 状态机 函数指针 区域 
    void* FSM_FUNCT(manual_on)(void * this_fsm);
    void* FSM_FUNCT(manual)(void * this_fsm);
    void* FSM_FUNCT(auto_hover)(void * this_fsm);
    void* FSM_FUNCT(cmd)(void * this_fsm);
    void* FSM_FUNCT(safe)(void * this_fsm);
    void* FSM_FUNCT(err)(void* this_fsm);  // 错误状态

    // sun: publish_* 为无确认的周期消息；request_* 通过服务发送需要响应的飞行器命令。
    void publish_vehicle_command_(uint16_t command, double param1 = 0.0, double param2 = 0.0);
    void publish_offboard_control_mode_();
    void request_vehicle_command_(VehicleCommandRequest request_type, uint16_t command,
                                  double param1 = 0.0, double param2 = 0.0);
    void response_callback_(VehicleCommandRequest request_type,
                            rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    bool process_vehicle_command_(VehicleCommandRequest request_type, uint16_t command,
                                  double param1, double param2, const char *mode_name);
    void reset_point_reference_(const Eigen::Vector3d &position);
    void update_point_reference_(double dt);
    // 仅在进入 CMD 的首周期调用并以当前 ROS 时刻作为轨迹 t=0 装载；
    // 多圈轨迹和离线 omtraj 模板在这里都只应用空间变换。
#if PX4CTRL_USE_OMMPC_PRIMARY_CONTROLLER
    px4ctrl::FiveTurnTrajectoryOptions five_turn_options_;
    OmTrajectoryResult five_turn_template_;
    bool five_turn_template_ready_{false};
    OmTrajectoryResult omtraj_template_;
    Eigen::Vector3d omtraj_template_initial_position_{Eigen::Vector3d::Zero()};
    double omtraj_template_initial_yaw_{0.0};
    bool omtraj_template_ready_{false};
    bool load_figure_eight_cmd_trajectory_();
    bool load_barrel_roll_cmd_trajectory_();
    bool load_five_turn_cmd_trajectory_();
    bool load_omtraj_cmd_trajectory_();
    bool load_minimum_snap_cmd_trajectory_();
#endif
    bool switch_to_offboard_mode_();
    bool switch_to_manual_mode_();
    
    
    
};

#endif
