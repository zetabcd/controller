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
#include <px4ctrl/controller.h>
#include <px4ctrl/ommpc.h>
#include <px4ctrl/input.h>
#include <px4ctrl/acados_nmpc.h>
#include <px4ctrl/trajectory.h>
#include <px4ctrl/takeoff_origin.h>
#include <px4ctrl/landing_reference.h>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

class PX4ControlNode;

// 顶层控制器编译开关；连续编号 0、1、2：
// 0=QuadControl, 1=OmMpcControl, 2=acados NMPC。
#ifndef PX4CTRL_PRIMARY_CONTROLLER
#define PX4CTRL_PRIMARY_CONTROLLER 1 // OMMPC 八字调参；本轮开始前为 2 (acados NMPC)
#endif

// Only controller setup and thrust-estimation behavior depend on this flag.
// Trajectory selection and reference generation are common to all controllers.
#if PX4CTRL_PRIMARY_CONTROLLER != 0 && PX4CTRL_PRIMARY_CONTROLLER != 1 && PX4CTRL_PRIMARY_CONTROLLER != 2
#error "Supported controllers: 0=QuadControl, 1=OmMpcControl, 2=acados NMPC"
#endif
#define PX4CTRL_USES_PREDICTIVE_CONTROLLER (PX4CTRL_PRIMARY_CONTROLLER != 0)

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
    AcadosNmpcControl controller;
#else
    // [LEGACY ACTIVE] 将上面的宏改为 0 即恢复原串级控制器。
    QuadControl controller;
#endif
    bool service_done;

    PX4CtrlFSM(PX4ControlNode &);
    void process();
    // Called before the main loop's armed/kill gates, including while disarmed.
    void update_flight_origin();
    void feed_land_detected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void reset_start_time();
    void set_init_ref();
    void set_hover_ref();
    void set_point_hover(const double &x, const double &y, const double &z);
    void set_manual_ref(const double &dt, bool reset_yaw_des=false);
    void set_manual_postion_ref(const double &dt, bool reset_yaw_des=false);
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
    bool prepare_cmd_trajectory();

    // sun: 这些发布器分别负责外部模式心跳、飞行器命令、控制设定值和调试信息。
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher;
    rclcpp::Publisher<ratectrl_msgs::msg::RatesThrustSetpoint>::SharedPtr rates_thrust_setpoint_publisher;
    rclcpp::Publisher<px4debug_msgs::msg::Px4ctrlDebug>::SharedPtr px4ctrldebug_publisher;
    

    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client;

private:
    void calculate_control_();
    void reset_controller_();
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
        DISARM,
    };

    // sun: control_sp_ 是本周期控制输出，ref_ 是由当前状态生成的统一参考状态。
    Control_Setpoint_t control_sp_;
    px4ctrl::TrajectoryPlayer trajectory_reference_;
    px4ctrl::TakeoffOrigin takeoff_origin_;
    px4ctrl::LandingReference landing_;
    Eigen::Vector3d hover_target_{Eigen::Vector3d::Zero()};
    double landing_start_time_{0.0}, land_received_time_{-1.0};
    double next_disarm_request_time_{0.0};
    uint64_t land_timestamp_{0}, land_start_timestamp_{0};
    bool land_detected_{false}, flight_was_armed_{false};
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
    std::shared_ptr<const px4ctrl::Trajectory> cmd_trajectory_;
    bool load_cmd_trajectory_();
    bool switch_to_offboard_mode_();
    bool switch_to_manual_mode_();
    void set_requested_landing_ref_();
    
    
    
};

#endif
