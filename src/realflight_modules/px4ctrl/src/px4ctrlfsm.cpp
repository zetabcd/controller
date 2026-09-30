#include <px4ctrl/sampled_trajectory.h>
#include "px4ctrl/input.h"
#include <Eigen/src/Core/Matrix.h>
#include <px4ctrl/px4ctrlfsm.h>
#include <px4ctrl/px4ctrl_node.h>
#include <uav_utils/geometry_utils.h>

#include <algorithm>
#include <utility>
#include <vector>

#define pi acos(-1)
#define deg2rad(x) x/180.0*pi

// sun: 状态机的安全层级为 manual_on（PX4 接管）-> manual/hover/cmd（外部控制）
// sun: -> safe（受控降落）-> err（框架故障）。每个状态函数在一次 process() 中只执行一步。

using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(PX4ControlNode& px4controlnode) : 
    rc_data(px4controlnode), sta_data(px4controlnode), bat_data(px4controlnode),
    sens_data(px4controlnode), att_data(px4controlnode), pose_data(px4controlnode),
#if PX4CTRL_PRIMARY_CONTROLLER != 2
    controller(px4controlnode),
#endif
    px4controlnode_(px4controlnode)
{
    // sun: procedure_list_ 的顺序必须与头文件 procedure_id_ 枚举一致，否则状态号会调用错误处理函数。
    start_time_ = px4controlnode_.get_clock()->now();
    last_time_ = px4controlnode_.get_clock()->now();

    procedure_list_[0] = std::bind(&PX4CtrlFSM::FSM_FUNCT(manual_on), this, std::placeholders::_1);
    procedure_list_[1] = std::bind(&PX4CtrlFSM::FSM_FUNCT(manual), this, std::placeholders::_1);
    procedure_list_[2] = std::bind(&PX4CtrlFSM::FSM_FUNCT(auto_hover), this, std::placeholders::_1);
    procedure_list_[3] = std::bind(&PX4CtrlFSM::FSM_FUNCT(cmd), this, std::placeholders::_1);
    procedure_list_[4] = std::bind(&PX4CtrlFSM::FSM_FUNCT(safe), this, std::placeholders::_1);
    procedure_list_[5] = std::bind(&PX4CtrlFSM::FSM_FUNCT(err), this, std::placeholders::_1);

    // 设置 状态机
    set_procedures(&fsm_, procedure_list_);
    set_data_entry(&fsm_, &rc_data);
#ifdef USE_WITHOUT_RC
    set_default_state(&fsm_, FSM_STATE(manual));
#else
    set_default_state(&fsm_, FSM_STATE(manual_on));
#endif  
    set_err_var(&fsm_, &err_code_);
    clr_fsm_error_flag(&fsm_);

    service_result_ = 0;
    service_done = false;

    record_state_data.set_zero();
}


void PX4CtrlFSM::process()
{
    // sun: 每周期先更新时间和悬停油门点，再步进 FSM；dt_ 同时供积分器和参考生成使用。
    rc_data.hover_percentage = px4controlnode_.param.motor.hover_percentage;
    now_time = px4controlnode_.get_clock()->now();
    t_ = (now_time - start_time_).seconds();
    dt_ = (now_time - last_time_).seconds();
    last_time_ = now_time;

    // 执行状态机，使其步进一次
    cur_state_ = run_state_machine_once(&fsm_);
    // manual/auto_hover/cmd/safe 都属于外部控制状态。心跳在
    // process() 中统一发布，保证状态处理函数提前返回时也不会中断。
    if (cur_state_ == FSM_STATE(manual) ||
        cur_state_ == FSM_STATE(auto_hover) ||
        cur_state_ == FSM_STATE(cmd) ||
        cur_state_ == FSM_STATE(safe))
    {
        publish_offboard_control_mode_();
    }
    // sun: 通用 FSM 错误标志表示状态索引或过程调用异常，读取后清除以免重复报告同一错误。
    if(is_fsm_error(&fsm_))
    {
        printf("Error when Stepping !\n");
        got_err_ = (int *)get_err_var(&fsm_);
        printf("Get Error Code :0x%x\n", *got_err_);
        clr_fsm_error_flag(&fsm_);
        // rclcpp::shutdown();
    }
    
    debug_msg.voltage = bat_data.voltage_filtered_v;
    debug_msg.state = get_curr_state(&fsm_);
    px4ctrldebug_publisher->publish(debug_msg);
#ifdef USE_WITHOUT_RC
    // sun: 无遥控编译模式用合成摇杆消息复用正常状态切换逻辑，避免维护另一套启动流程。
    auto joy_empty_msg = std::make_unique<joy_msgs::msg::JoyStick>();
    joy_empty_msg->aux1 = 1.0;
    joy_empty_msg->aux2 = -1.0;
    double enter_auto_hover_after_launch = 0.1; // 秒
    double enter_cmd_after_hover = 2.0; // 秒
    if (px4controlnode_.param.tuning.attitude_loop || px4controlnode_.param.tuning.angular_rate_loop)
    {
        enter_auto_hover_after_launch = 3.0;
    }
    if (t_ >= enter_auto_hover_after_launch)
    {
        // 切换到 auto_hover
        joy_empty_msg->aux2 = 0.0;

        if (!px4controlnode_.param.tuning.attitude_loop && 
            !px4controlnode_.param.tuning.angular_rate_loop){
                if (t_ > (enter_auto_hover_after_launch + enter_cmd_after_hover))
                {
                    // 切换到 cmd
                    joy_empty_msg->aux2 = 1.0;
                }
            }
    }
    rc_data.feed(std::move(joy_empty_msg));//这里定义没有RC的时候自己给rc回调函数赋值
#endif
}

void PX4CtrlFSM::reset_start_time()
{
    // sun: 重置任务相对时间，后续轨迹以新的状态进入时刻作为 t=0。
    start_time_ = px4controlnode_.get_clock()->now();
}
/* MANUAL(ONBOARD)模式 */ 
void* PX4CtrlFSM::FSM_FUNCT(manual_on)(void * this_fsm)
{   
    // sun: manual_on 表示 PX4 本机模式仍掌握执行器；控制节点仅监视输入并等待切入 OFFBOARD。
    /* 状态切换 */ 
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    if (pd->aux1_changed) {
        if (pd->aux1 == GEARS::UP) {
            pending_mode_switch_ = ModeSwitchTarget::OFFBOARD;
        } else if (pending_mode_switch_ == ModeSwitchTarget::OFFBOARD) {
            //这种模式怎么进来的？根本不可能形成这种状态（不对我错了，这种情况是要假设FSM的频率比遥控器高的，
            // 这个时候拨到down，但是offboard又没有执行完，持续一个时序）
            // OFFBOARD 请求可能已经在 PX4 端执行；排队切回 MANUAL 来收敛到最新开关位置。
            // ModeSwitchTarget::MANUAL是指px4的状态，也即对应FSM的manual_on而不是manual（切记！！！）
            // 但这里有个bug，rc的频率远低于fsm，一个跳变沿会被执行多次，这里也就直接跳到ModeSwitchTarget::NONE，那么下面就无法回到MANUAL
            pending_mode_switch_ = ModeSwitchTarget::MANUAL;
        } else {
            pending_mode_switch_ = ModeSwitchTarget::NONE;
        }
        if (pd->aux1 == GEARS::DOWN) {
            set_default_state(&fsm_, FSM_STATE(manual_on));
        }
        pd->aux1_changed = false;
    }
    if (pending_mode_switch_ == ModeSwitchTarget::OFFBOARD) {
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[33m[px4ctrl] Try to switch offboard.\033[0m");
        // sun: 切入 OFFBOARD 前要求位置、姿态和 IMU 都新鲜，避免用默认状态闭环。
        if (!pose_is_received(now_time)) {
            RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject MANUAL(OFF). No pose!\033[0m");
            pending_mode_switch_ = ModeSwitchTarget::NONE;
        } else if (!att_is_received(now_time)) {
            RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject MANUAL(OFF). No attitude!\033[0m");
            pending_mode_switch_ = ModeSwitchTarget::NONE;
        } else if (!sens_is_received(now_time)) {
            RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject MANUAL(OFF). No imu!\033[0m");
            pending_mode_switch_ = ModeSwitchTarget::NONE;
        } else {
            // 请求等待期间持续预发送心跳；异步响应返回后无需再次拨动开关。
            publish_offboard_control_mode_();
            if (switch_to_offboard_mode_()) {
                pending_mode_switch_ = ModeSwitchTarget::NONE;
                set_last_state((FSM *)this_fsm);
                set_next_state((FSM *)this_fsm, FSM_STATE(manual));
                RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(ON) --> MANUAL(OFF)\033[0m");
#if !PX4CTRL_USES_PREDICTIVE_CONTROLLER
                // [LEGACY QuadControl] 在线推力映射复位。
                controller.resetThrustMapping(px4controlnode_.param);
#endif
                return NULL;
            }
        }
    }
    // 这里的思维太复杂了，嵌套太严重了。switch_to_manual_mode_看似每次执行，但是它在里面也判定是否为MANUAL才执行
    // pending_mode_switch_ != MANUAL：不会调用 switch_to_manual_mode_()
    // 似乎这一套算法在高频对低频的时候无法使用，因为以下请求必须不止一次请求。但因为FSM频率太高使得它只有一次机会请求。
    if (pending_mode_switch_ == ModeSwitchTarget::MANUAL && switch_to_manual_mode_()) {
        pending_mode_switch_ = ModeSwitchTarget::NONE;
    }

    //正常不进入if的时候执行，也就是要控制不动的时候执行下面这段代码
#ifndef USE_WITHOUT_RC
    /* 故障处理 */
    if (!rc_is_received(px4controlnode_.get_clock()->now()))
    {
        // set_last_state((FSM *)this_fsm);
        set_next_state((FSM *)this_fsm, FSM_STATE(err));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is disconnected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(ON) --> ERROR\033[0m");
    }
#endif
    /* 任务 */ 
    control_sp_ = Control_Setpoint_t();
    publish_rates_thrust_setpoint();
    return NULL;
    
}

/* MANUAL(OFFBOARD)模式 */ 
void* PX4CtrlFSM::FSM_FUNCT(manual)(void * this_fsm)
{
    // sun: manual 是 OFFBOARD 下的遥控姿态模式；首次进入时锁定现场状态并清控制器历史。
    if (get_last_state((FSM *)this_fsm) != FSM_STATE(manual))
    {
        // Leaving CMD clears the active source before local mode references resume.
        trajectory_reference_.clear();
        set_init_ref();
        record_position();
        reset_controller_();
#ifndef USE_WITHOUT_RC
        set_manual_ref(dt_, true);
#endif
        set_last_state((FSM *)this_fsm);
    }
    /* 状态切换 */ 
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    // std::cout << "pd->aux1:" << pd->aux1 << std::endl;
    // std::cout << "pd->aux2:" << pd->aux2 << std::endl;
    if (pd->aux1_changed) {
        if (pd->aux1 == GEARS::DOWN) {
            pending_mode_switch_ = ModeSwitchTarget::MANUAL;
        } else if (pending_mode_switch_ == ModeSwitchTarget::MANUAL) {
            // MANUAL 请求在途时用户拨回：等待旧响应结束后重新请求 OFFBOARD。
            pending_mode_switch_ = ModeSwitchTarget::OFFBOARD;
        } else {
            pending_mode_switch_ = ModeSwitchTarget::NONE;
        }
    }
    if (pending_mode_switch_ == ModeSwitchTarget::MANUAL && switch_to_manual_mode_()) {
        pending_mode_switch_ = ModeSwitchTarget::NONE;
        set_next_state((FSM *)this_fsm, FSM_STATE(manual_on));
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(OFF) --> MANUAL(ON)\033[0m");
        return NULL;
    }
    if (pending_mode_switch_ == ModeSwitchTarget::OFFBOARD && switch_to_offboard_mode_()) {
        pending_mode_switch_ = ModeSwitchTarget::NONE;
    }
    if(pd->aux2_changed){
        switch (pd->aux2){
            case GEARS::MID:
                // sun: 悬停属于位置闭环，除数据新鲜度外还要求 PX4 的位置有效标志成立。
                if (!pose_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject AUTO_HOVER. No pose!\033[0m");
                    break;
                }
                if (!att_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject AUTO_HOVER. No attitude!\033[0m");
                    break;
                }
                if (!sens_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject AUTO_HOVER. No imu!\033[0m");
                    break;
                }
                if (!pose_is_valid(pose_data))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject AUTO_HOVER. Local position is invalid!\033[0m");
                    break;
                }
                //  aux2_has_downed 是历史锁存标志，不代表当前处于 DOWN
                // 这里还比较关键，防止的一种情况：从UP拨到MID。也就是防止一开始拨杆不在默认位置上。
                if (pd->aux2_has_downed)
                {
                    // set_last_state((FSM *)this_fsm);
                    set_next_state((FSM *)this_fsm, FSM_STATE(auto_hover));
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(OFF) --> AUTO_HOVER\033[0m");
                }
                return NULL;
                break;
            default:
                break;
        }
    }
#ifndef USE_WITHOUT_RC
    /* 故障处理 */
    if (!rc_is_received(px4controlnode_.get_clock()->now()) || (pd->aux6 == GEARS::UP))
    {
        // set_last_state((FSM *)this_fsm);
        set_next_state((FSM *)this_fsm, FSM_STATE(safe));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is disconnected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(OFF) --> SAFE\033[0m");
    }
#endif
    /* 任务 */
#ifdef USE_WITHOUT_RC
    control_sp_ = Control_Setpoint_t();
    control_sp_.thrust = px4controlnode_.param.uav.mass * px4controlnode_.param.gra;
    control_sp_.bodyrates = Eigen::Vector3d::Zero();
#else
    set_manual_ref(dt_,false);
    calculate_control_();
#endif
    publish_rates_thrust_setpoint();
    return NULL;
}

/* AUTO_HOVER模式 */ 
void* PX4CtrlFSM::FSM_FUNCT(auto_hover)(void * this_fsm)
{
    // sun: auto_hover 锁定进入状态时的位置和航向，并在稳定工况下持续辨识推力映射。
    /*  刚进入状态 */
    if (get_last_state((FSM *)this_fsm) != FSM_STATE(auto_hover))
    {
        // Non-CMD modes use their own local reference, never the previous trajectory.
        trajectory_reference_.clear();
        set_init_ref();
        record_position();
        reset_point_reference_(record_state_data.p + record_state_data.v * 0.3);
        reset_controller_();
#if !PX4CTRL_USES_PREDICTIVE_CONTROLLER
        // [LEGACY QuadControl] 悬停阶段复位推力映射。
        controller.resetThrustMapping(px4controlnode_.param);
#endif
        set_last_state((FSM *)this_fsm);
    }
    /* 状态切换 */ 
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    if (pd->aux1_changed) {
        if (pd->aux1 == GEARS::DOWN) {
            pending_mode_switch_ = ModeSwitchTarget::MANUAL;
        } else if (pending_mode_switch_ == ModeSwitchTarget::MANUAL) {
            pending_mode_switch_ = ModeSwitchTarget::OFFBOARD;
        } else {
            pending_mode_switch_ = ModeSwitchTarget::NONE;
        }
    }
    if (pending_mode_switch_ == ModeSwitchTarget::MANUAL && switch_to_manual_mode_()) {
        pending_mode_switch_ = ModeSwitchTarget::NONE;
        set_next_state((FSM *)this_fsm, FSM_STATE(manual_on));
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] AUTO_HOVER --> MANUAL(ON)\033[0m");
        return NULL;
    }
    if (pending_mode_switch_ == ModeSwitchTarget::OFFBOARD && switch_to_offboard_mode_()) {
        pending_mode_switch_ = ModeSwitchTarget::NONE;
    }
    if(pd->aux2_changed){
        switch (pd->aux2){
            case GEARS::UP:
                // sun: CMD 轨迹会用到 p/v/a 多阶反馈，因此再次完整检查估计数据。
                if (!pose_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject CMD_CTRL. No pose!\033[0m");
                    break;
                }
                if (!att_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject CMD_CTRL. No attitude!\033[0m");
                    break;
                }
                if (!sens_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject CMD_CTRL. No imu!\033[0m");
                    break;
                }
                if (!pose_is_valid(pose_data))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject CMD_CTRL. Local position is invalid!\033[0m");
                    break;
                }
                if (get_last_state((FSM *)this_fsm) == FSM_STATE(cmd)) // 目标丢失切回来的情况
                {
                    break;
                }

                set_next_state((FSM *)this_fsm, FSM_STATE(cmd));
                RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] AUTO_HOVER --> CMD_CTRL\033[0m");
                return NULL;
                break;
            case GEARS::DOWN:
                // aux2 只切换 Offboard 内部子模式；PX4 仍保持 Offboard。
                set_next_state((FSM *)this_fsm, FSM_STATE(manual));
                RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] AUTO_HOVER --> MANUAL(OFF)\033[0m");
                return NULL;
                break;
            default:
                break;
        }
    }
#ifndef USE_WITHOUT_RC
    /* 故障处理 */
    if (!rc_is_received(px4controlnode_.get_clock()->now()) || (pd->aux6 == GEARS::UP))
    {
        set_next_state((FSM *)this_fsm, FSM_STATE(safe));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is disconnected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] AUTO_HOVER --> SAFE\033[0m");
    }
#endif
    /* 任务 */
    // [LEGACY QuadControl] 先用当前 IMU 更新推力模型；OMMPC 不使用该映射。
    // set_hover_ref();
    set_point_hover(0,0,0.5);
    // set_manual_postion_ref(dt_,false);
#if !PX4CTRL_USES_PREDICTIVE_CONTROLLER
    controller.estimateThrustModel(sens_data.a);
#endif
    calculate_control_();

    // RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31mJust for debug!!!\033[0m");

    publish_rates_thrust_setpoint();

    return NULL;
}

/* CMD模式 */ 
// 不进行在线推力系数辨识，使用悬停时的值
void* PX4CtrlFSM::FSM_FUNCT(cmd)(void * this_fsm)
{   
    // RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31mJust for debug11111111111111!!!\033[0m");
    // The template is prepared before control starts. Activate it once in the
    // measured local frame, then evaluate H+1 points at exact prediction times.
    if (get_last_state((FSM *)this_fsm) != FSM_STATE(cmd))
    {
        record_position();
        set_hover_ref();
        reset_controller_();
        if (!load_cmd_trajectory_()) {
            set_next_state((FSM *)this_fsm, FSM_STATE(auto_hover));
            return NULL;
        }
        set_last_state((FSM *)this_fsm);
    }
    /* 状态切换 */ 
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    if (pd->aux1_changed) {
        if (pd->aux1 == GEARS::DOWN) {
            pending_mode_switch_ = ModeSwitchTarget::MANUAL;
        } else if (pending_mode_switch_ == ModeSwitchTarget::MANUAL) {
            pending_mode_switch_ = ModeSwitchTarget::OFFBOARD;
        } else {
            pending_mode_switch_ = ModeSwitchTarget::NONE;
        }
    }
    if (pending_mode_switch_ == ModeSwitchTarget::MANUAL && switch_to_manual_mode_()) {
        pending_mode_switch_ = ModeSwitchTarget::NONE;
        set_next_state((FSM *)this_fsm, FSM_STATE(manual_on));
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> MANUAL(ON)\033[0m");
        return NULL;
    }
    if (pending_mode_switch_ == ModeSwitchTarget::OFFBOARD && switch_to_offboard_mode_()) {
        pending_mode_switch_ = ModeSwitchTarget::NONE;
    }
    if(pd->aux2_changed){
        switch (pd->aux2){
            case GEARS::MID:
                // UP -> MID 回到悬停，不改变 PX4 Offboard 模式。
                set_next_state((FSM *)this_fsm, FSM_STATE(auto_hover));
                RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> AUTO_HOVER\033[0m");
                return NULL;
                break;
            case GEARS::DOWN:
                // 允许三段开关跨档采样：UP -> DOWN 直接回到 Offboard 手动。
                set_next_state((FSM *)this_fsm, FSM_STATE(manual));
                RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> MANUAL(OFF)\033[0m");
                return NULL;
                break;
            
            default:
                break;
        }
    }
#ifndef USE_WITHOUT_RC
    /* 故障处理 */
    if (!rc_is_received(px4controlnode_.get_clock()->now()) || (pd->aux6 == GEARS::UP))
    {
        // set_last_state((FSM *)this_fsm);
        set_next_state((FSM *)this_fsm, FSM_STATE(safe));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is disconnected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> SAFE\033[0m");
    }
#endif
    /* 任务 */
#if !PX4CTRL_USES_PREDICTIVE_CONTROLLER
    // [LEGACY QuadControl] 原解析 8 字轨迹入口完整保留；切换宏为 0 后自动启用。
    px4controlnode_.init_param();
#endif
    calculate_control_();
    publish_rates_thrust_setpoint();
    return NULL;
}

/* 安全模式 */
void* PX4CtrlFSM::FSM_FUNCT(safe)(void* this_fsm)
{
    // sun: safe 先保持进入点附近姿态；当速度估计无效或速度已降到阈值内，再转为恒速下降。
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    static bool executed = false;
    /*  刚进入状态 */
    if (get_last_state((FSM *)this_fsm) != FSM_STATE(safe))
    {
        // [OMMPC] 安全/降落参考必须覆盖 CMD 轨迹源。
        trajectory_reference_.clear();
        record_position();
        reset_controller_();
#if !PX4CTRL_USES_PREDICTIVE_CONTROLLER
        // [LEGACY QuadControl] 安全模式恢复推力映射。
        controller.resetThrustMapping(px4controlnode_.param);
#endif
        set_last_state((FSM *)this_fsm);
        executed = false;
    }
    if (rc_is_received(px4controlnode_.get_clock()->now()) && (pd->aux6 == GEARS::DOWN))
    {
        set_last_state((FSM *)this_fsm);
        set_next_state((FSM *)this_fsm, FSM_STATE(manual));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is connected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> MANUAL(OFF)\033[0m");
        px4controlnode_.init_param();
        return NULL;
    }

    set_hover_ref();
    if (!pose_data.v_xy_valid || pose_data.v.norm() < 3.0) // 速度小于3m/s或者速度估计无效时
    {
        px4controlnode_.init_param();
        if (!executed) {
            // sun: 下降起点只记录一次，否则每周期重置时间会使下降参考始终停在原处。
            record_position();
            executed = true;
        }
        set_land_ref();
    }
    calculate_control_();
    publish_rates_thrust_setpoint();

    return NULL;
}

bool PX4CtrlFSM::prepare_cmd_trajectory()
{
    try {
        const auto &p = px4controlnode_.param;
        const auto type = px4controlnode_.get_parameter("trajectory.type").as_string();
        if (type == "omtraj") {
            cmd_trajectory_ = px4ctrl::loadOmTrajectoryReference(
                px4controlnode_.get_parameter("trajectory.omtraj.file").as_string(), p.gra);
        } else {
            px4ctrl::AnalyticTrajectoryOptions o;
            if (type == "horizontal_circle") {o.path = px4ctrl::AnalyticPath::HorizontalCircle;}
            else if (type == "vertical_circle") {o.path = px4ctrl::AnalyticPath::VerticalCircle;}
            else if (type == "helix") {o.path = px4ctrl::AnalyticPath::Helix;}
            else if (type == "figure_eight") {o.path = px4ctrl::AnalyticPath::FigureEight;}
            else {throw std::invalid_argument("Unknown trajectory.type: " + type);}
            o.gravity = p.gra;
            const auto read = [&](const std::string &key, double &value) {
                px4controlnode_.get_parameter("trajectory." + key, value);
            };
            read("takeoff_height", o.takeoff_height);
            read("takeoff_duration", o.takeoff_duration);
            read("settle_duration", o.settle_duration);
            read(type + ".radius", o.radius);
            o.turns = static_cast<int>(px4controlnode_.get_parameter("trajectory." + type + ".turns").as_int());
            if (type == "horizontal_circle" || type == "figure_eight") {
                read(type + ".speed", o.speed);
                read(type + ".ramp_duration", o.ramp_duration);
            } else {
                read(type + ".centripetal_g", o.centripetal_g);
                read(type + ".entry_duration", o.entry_duration);
                read(type + ".exit_duration", o.exit_duration);
                read(type + ".entry_distance", o.entry_distance);
                read(type + ".exit_distance", o.exit_distance);
                read(type + ".connector_height", o.connector_height);
            }
            if (type == "helix") {
                read("helix.pitch", o.pitch);
                read("helix.axis_transition_duration", o.axis_transition_duration);
            }
            if (type == "figure_eight") {
                read("figure_eight.length", o.eight_length);
                read("figure_eight.width", o.eight_width);
            }
            cmd_trajectory_ = std::make_shared<px4ctrl::AnalyticTrajectory>(o);
        }
        px4ctrl::TrajectoryLimits limits;
        limits.gravity = p.gra; limits.mass = p.uav.mass;
        limits.inertia = {p.uav.Jvx, p.uav.Jvy, p.uav.Jvz};
        limits.arm = p.uav.l; limits.arm_angle = p.uav.beta_deg * 3.14159265358979323846 / 180.0;
        limits.torque_to_thrust = p.motor.cq0 / p.motor.ct0;
        limits.motor_min = p.motor.u_min;
        const double motor_fraction = px4controlnode_.get_parameter("trajectory.limits.motor_fraction").as_double();
        if (!std::isfinite(motor_fraction) || motor_fraction <= 0 || motor_fraction > 1) {
            throw std::invalid_argument("trajectory.limits.motor_fraction must be in (0,1]");
        }
        limits.motor_max = p.motor.u_max * motor_fraction;
        limits.thrust_min = 4.0 * limits.motor_min / limits.mass;
        limits.thrust_max = 4.0 * limits.motor_max / limits.mass;
#if PX4CTRL_PRIMARY_CONTROLLER != 0
        limits.rate_max = controller.options().body_rate_max;
        limits.thrust_min = std::max(limits.thrust_min, controller.options().thrust_acceleration_min);
        limits.thrust_max = std::min(limits.thrust_max, controller.options().thrust_acceleration_max);
#endif
        limits.angular_acceleration_max = px4controlnode_.get_parameter("trajectory.limits.angular_acceleration").as_double();
        limits.minimum_relative_altitude = px4controlnode_.get_parameter("trajectory.limits.minimum_relative_altitude").as_double();
        const auto audit = px4ctrl::auditTrajectory(*cmd_trajectory_, limits);
        if (!audit.valid) {
            throw std::invalid_argument(audit.reason + " at t=" + std::to_string(audit.first_failure_time));
        }
        RCLCPP_INFO(px4controlnode_.get_logger(),
            "Trajectory %s ready: %.3f s, speed %.2f m/s, thrust [%.2f, %.2f] m/s2, "
            "rate %.2f rad/s, alpha %.2f rad/s2, static motors [%.2f, %.2f] N; sampled nominal audit only",
            type.c_str(), cmd_trajectory_->duration(), audit.max_speed, audit.min_thrust,
            audit.max_thrust, audit.max_rate, audit.max_angular_acceleration, audit.min_motor, audit.max_motor);
        return true;
    } catch (const std::exception &error) {
        cmd_trajectory_.reset();
        RCLCPP_ERROR(px4controlnode_.get_logger(), "Cannot prepare trajectory: %s", error.what());
        return false;
    }
}

bool PX4CtrlFSM::load_cmd_trajectory_()
{
    if (!cmd_trajectory_) {return false;}
    trajectory_reference_.start(cmd_trajectory_, px4controlnode_.get_clock()->now().seconds(),
        record_state_data.p, get_yaw_from_quaternion(record_state_data.q));
    return true;
}

/* 故障模式 */
void* PX4CtrlFSM::FSM_FUNCT(err)(void* this_fsm)// 错误状态
{
    // sun: err 是 FSM 框架级兜底状态，通过共享错误码把异常通知给 process()。
    int *err_var;


    // 通知 调用者 有错误发生
    set_fsm_error_flag((FSM *)this_fsm);

    // 把 错误值 设进 容器中（如果容器存在）
    err_var = (int *)get_err_var((FSM *)this_fsm);
    if(err_var) 
        *err_var = 0xff;
    return NULL;
}

//---------------------------------------------------------------------------------------------------------------
void PX4CtrlFSM::set_init_ref()
{
    // sun: 统一填充所有参考阶次，避免状态切换后沿用上一模式未覆盖的字段。
	Ref_State_t ref;
	ref.p = Eigen::Vector3d::Zero();
	ref.v = Eigen::Vector3d::Zero();
	ref.a = Eigen::Vector3d::Zero();
	ref.j = Eigen::Vector3d::Zero();
    ref.s = Eigen::Vector3d::Zero();
    ref.q = Eigen::Quaterniond::Identity();
	ref.yaw_rate = 0.0;
    ref.throttle = 0.0;
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;
}
void PX4CtrlFSM::set_hover_ref()
{
    // sun: 用进入状态时的速度做 0.3 s 前视补偿，使悬停点位于当前运动趋势前方，减小急停冲击。
	Ref_State_t ref;
	ref.p = record_state_data.p + record_state_data.v * 0.3;
	ref.v = Eigen::Vector3d::Zero();
	ref.a = Eigen::Vector3d::Zero();
    ref.q = yaw_to_quaternion(get_yaw_from_quaternion(record_state_data.q));
    ref.yaw_rate = 0.0;    
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;
}

void PX4CtrlFSM::reset_point_reference_(const Eigen::Vector3d &position)
{
    point_target_ = position;
    point_ref_position_ = position;
    point_ref_velocity_.setZero();
    point_ref_yaw_ = get_yaw_from_quaternion(record_state_data.q);
    px4controlnode_.get_parameter("trajectory.point_to_point.max_speed", point_max_speed_);
    px4controlnode_.get_parameter(
        "trajectory.point_to_point.max_acceleration", point_max_acceleration_);
    point_max_speed_ = std::max(0.05, point_max_speed_);
    point_max_acceleration_ = std::max(0.05, point_max_acceleration_);
    point_ref_initialized_ = true;
}

void PX4CtrlFSM::update_point_reference_(double dt)
{
    if (!point_ref_initialized_) {
        reset_point_reference_(pose_data.p);
    }

    dt = std::clamp(dt, 0.0, 0.05);
    const Eigen::Vector3d error = point_target_ - point_ref_position_;
    Eigen::Vector3d velocity_des = error / std::max(dt, 1.0e-3);
    const double braking_speed = std::sqrt(
        2.0 * point_max_acceleration_ * error.norm());
    const double speed_limit = std::min(point_max_speed_, braking_speed);
    if (velocity_des.norm() > speed_limit && velocity_des.norm() > 1.0e-9) {
        velocity_des = velocity_des.normalized() * speed_limit;
    }

    Eigen::Vector3d dv = velocity_des - point_ref_velocity_;
    const double dv_max = point_max_acceleration_ * dt;
    if (dv.norm() > dv_max && dv.norm() > 1.0e-9) {
        dv *= dv_max / dv.norm();
    }

    const Eigen::Vector3d velocity_old = point_ref_velocity_;
    point_ref_velocity_ += dv;
    point_ref_position_ += point_ref_velocity_ * dt;
    ref_.p = point_ref_position_;
    ref_.v = point_ref_velocity_;
    if (dt > 1.0e-6) {
        ref_.a = (point_ref_velocity_ - velocity_old) / dt;
    } else {
        ref_.a.setZero();
    }
}

void PX4CtrlFSM::set_point_hover(const double &x, const double &y, const double &z)
{
    // sun: 用进入状态时的速度做 0.3 s 前视补偿，使悬停点位于当前运动趋势前方，减小急停冲击。
    Eigen::Vector3d point(x,y,z);
	Ref_State_t ref;
	ref.p = point;
	ref.v = Eigen::Vector3d::Zero();
	ref.a = Eigen::Vector3d::Zero();
    ref.q = yaw_to_quaternion(get_yaw_from_quaternion(record_state_data.q));
    ref.yaw_rate = 0.0;    
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;
}

void PX4CtrlFSM::set_land_ref()
{
    // sun: 降落参考保持水平位置和进入时航向，只沿 ENU z 轴以 0.3 m/s 匀速下降。
	Ref_State_t ref;
    auto start_time = record_state_data.time;
    auto now_time = px4controlnode_.get_clock()->now();
    double t = (now_time-start_time).seconds();
    
    // 速度减到0
    double vmax = 0.3;
    ref.p << 0.0, 0.0, -vmax*t;
    ref.p += record_state_data.p + record_state_data.v * 0.3;
    // des.p << 0.0, 0.0, 0.0;
    ref.v << 0.0, 0.0, -vmax;
	ref.a << 0.0, 0.0, 0.0;
	ref.j << 0.0, 0.0, 0.0;
    ref.s << 0.0, 0.0, 0.0;
    ref.q = yaw_to_quaternion(get_yaw_from_quaternion(record_state_data.q));
	ref.yaw_rate = 0.0;
    ref.yaw_accel = 0.0;
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;
}

void PX4CtrlFSM::record_position()
{
    // sun: 记录位置、速度、加速度、姿态和时间，作为后续悬停/降落/轨迹的公共初始条件。
    record_state_data.time = px4controlnode_.get_clock()->now();
	record_state_data.p = pose_data.p;
    record_state_data.v = pose_data.v;
    record_state_data.a = sens_data.a;
    record_state_data.q = att_data.q;
    record_state_data.yaw = get_yaw_from_quaternion(att_data.q);
}

void PX4CtrlFSM::set_manual_ref(const double &dt, bool reset_yaw_des)
{
    // sun: 遥控滚转/俯仰映射到 ±90°，偏航通道映射到 ±90°/s，油门映射到 [0,1]。
    rclcpp::Time time_now = px4controlnode_.get_clock()->now();

    double rc_roll = rc_data.roll*deg2rad(90) * (px4controlnode_.param.rc_reverse.roll ? 1 : -1);
    double rc_pitch = rc_data.pitch*deg2rad(90) * (px4controlnode_.param.rc_reverse.pitch ? 1 : -1);
    double manual_throttle = (rc_data.throttle * (px4controlnode_.param.rc_reverse.throttle ? 1 : -1) + 1)/2.0 ; // [0,1]
    // std::cout<<"manual_throttle:  "<<manual_throttle<<std::endl;
    double manual_yawrate = rc_data.yaw * deg2rad(90) * (px4controlnode_.param.rc_reverse.yaw ? 1 : -1);
    
    // sun: 大油门时把航向参考重新贴合当前记录航向，避免积分航向误差造成突转。
    if (manual_throttle > 0.9) {
		reset_yaw_des = true;
	}
    // 确保绝对航向误差不积累
    if (reset_yaw_des){
        manual_yaw_ = record_state_data.yaw;
    }else{
        manual_yaw_ = normalize_angle(manual_yaw_ + dt*manual_yawrate);
    }

    Eigen::Vector3d axis(rc_roll, rc_pitch, 0.0); // 旋转轴（单位向量）
    // sun: 将滚转/俯仰摇杆组成倾转轴角，再左乘航向四元数，实现倾转与偏航解耦。
    double angle = axis.norm(); // 旋转角度（弧度）
    Eigen::Quaterniond q_rp = Eigen::Quaterniond::Identity();
    if (angle > 1.0e-8) {
        axis /= angle;
        q_rp = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    }
    Eigen::Quaterniond q_yaw(cos(manual_yaw_ / 2.f), 0.f, 0.f, sin(manual_yaw_ / 2.f));
    
    Eigen::Quaterniond manual_q = q_yaw*q_rp;

    Ref_State_t ref;
	ref.p << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
	ref.v << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
	ref.a << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
    ref.j << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
    ref.s << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
	ref.q = manual_q;
	ref.yaw_rate = manual_yawrate;
    ref.throttle = manual_throttle;
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;

}


//遥控器位置控制逻辑实现
//有摇杆量的时候就失效位置控制而转向速度控制，然后在不控制的时候立马采集此时的传感器数据作为位置控制的期望值
void PX4CtrlFSM::set_manual_postion_ref(const double &dt, bool reset_yaw_des)
{
    const double deadzone = 0.1;
    const auto apply_deadzone = [deadzone](double value) {
        if (std::abs(value) <= deadzone) return 0.0;
        return std::copysign(
            (std::abs(value) - deadzone) / (1.0 - deadzone), value);
    };

    if (!point_ref_initialized_) {
        reset_point_reference_(pose_data.p);
    }

    // 平移摇杆定义在机头水平坐标系，再仅绕当前航向旋转到 ENU。
    const double yaw = get_yaw_from_quaternion(att_data.q);
    Eigen::Vector3d velocity_body(
        apply_deadzone(rc_data.pitch),
        apply_deadzone(-rc_data.roll),
        apply_deadzone(rc_data.throttle));
    velocity_body *= point_max_speed_;
    const Eigen::Vector3d velocity_world =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * velocity_body;
    point_target_ += velocity_world * std::clamp(dt, 0.0, 0.05);

    const double yaw_rate = apply_deadzone(rc_data.yaw) * deg2rad(90.0) *
        (px4controlnode_.param.rc_reverse.yaw ? 1.0 : -1.0);
    point_ref_yaw_ = reset_yaw_des ? yaw :
        normalize_angle(point_ref_yaw_ + yaw_rate * dt);

    ref_ = Ref_State_t();
    update_point_reference_(dt);
    ref_.q = yaw_to_quaternion(point_ref_yaw_);
    ref_.yaw_rate = yaw_rate;
    ref_.yaw_accel = 0.0;
    ref_.fsm_state = get_curr_state(&fsm_);
    ref_.flag_valid_p = true;
    ref_.flag_valid_v = true;
    ref_.flag_valid_a = true;
}


//------------------------------------------------------------------------------------------------------------
bool PX4CtrlFSM::rc_is_received(const rclcpp::Time &now_time)
{
    // sun: 遥控允许较宽的 1.5 s 超时，位置/姿态等闭环反馈使用更严格的 0.5 s。
	return (now_time - rc_data.rcv_stamp).seconds() < 1.5; //param.msg_timeout.rc = 0.5s

}

/* 判断初始状态切换摇杆是否在最下面 */
bool PX4CtrlFSM::rc_is_downinit()
{
    return (rc_data.aux2 == GEARS::DOWN && rc_data.aux1 == GEARS::DOWN);
}

/* 判断即停开关 */
bool PX4CtrlFSM::rc_is_kill()
{
    return rc_data.aux4 == GEARS::UP;
}

/*  判断解锁开关 */
bool PX4CtrlFSM::rc_is_armed()
{
    static GEARS last_aux3 = rc_data.aux3;
    if (rc_data.aux3 == GEARS::UP)
    {
        if (last_aux3 == GEARS::DOWN || last_aux3 == GEARS::MID){
            last_aux3 = rc_data.aux3;
            return true;
        }
        else{
            return false;
        }
    }else{
        last_aux3 = rc_data.aux3;
        return false;
    }
}
/* 判断解锁 */
bool PX4CtrlFSM::vs_is_armed()
{
    return sta_data.arming_state == 2;
}

bool PX4CtrlFSM::pose_is_valid(const LocalPose_Data_t &pose)
{   
	return (pose.xy_valid && pose.z_valid && pose.v_xy_valid && pose.v_z_valid);
}

bool PX4CtrlFSM::pose_is_received(const rclcpp::Time &now_time)
{   
	return (now_time - pose_data.rcv_stamp).seconds() < 0.5;
}

bool PX4CtrlFSM::att_is_received(const rclcpp::Time &now_time)
{
	return (now_time - att_data.rcv_stamp).seconds() < 0.5;
}

bool PX4CtrlFSM::sens_is_received(const rclcpp::Time &now_time)
{
#if PX4CTRL_USE_FILTERED_IMU
	return sens_data.filtered_imu_is_received(now_time);
#else
	return (now_time - sens_data.rcv_stamp).seconds() < 0.5;
#endif
}

bool PX4CtrlFSM::bat_is_received(const rclcpp::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).seconds() < 0.5;
}

bool PX4CtrlFSM::recv_new_pose()
{
    // sun: 读取后立即清零，提供“每条位置消息最多触发一次处理”的消费语义。
	if (pose_data.recv_new_msg)
	{
		pose_data.recv_new_msg = false;
		return true;
	}
	return false;
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void PX4CtrlFSM::publish_vehicle_command_(uint16_t command, double param1, double param2)
{
	// sun: PX4 时间戳单位为微秒；source/target 均设为系统 1、组件 1，并标记命令来自外部。
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->px4controlnode_.get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher->publish(msg);
}

bool PX4CtrlFSM::switch_to_offboard_mode_(){
#ifdef SIMULATION
    // sun: 仿真器不实现 PX4 模式服务，直接视为切换成功。
    return true;
#endif
    return process_vehicle_command_(
        VehicleCommandRequest::OFFBOARD,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0,
        "offboard");
}
bool PX4CtrlFSM::switch_to_manual_mode_(){
#ifdef SIMULATION
    return true;
#endif
    return process_vehicle_command_(
        VehicleCommandRequest::MANUAL,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 1.0,
        "manual");
}
bool PX4CtrlFSM::arm()
{
    return process_vehicle_command_(
        VehicleCommandRequest::ARM,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0,
        "arm");
}

bool PX4CtrlFSM::process_vehicle_command_(
    VehicleCommandRequest request_type, uint16_t command,
    double param1, double param2, const char *mode_name)
{
    if (service_done) {
        // 可能在用户撤销切换后才收到旧响应；只有类型一致的调用来判断它。
        const bool response_matches = completed_service_request_ == request_type;
        service_done = false;
        completed_service_request_ = VehicleCommandRequest::NONE;
        if (response_matches) {
            if (service_result_ == 0) {
                return true;
            }
            RCLCPP_ERROR(
                px4controlnode_.get_logger(),
                "\033[31mFailed to enter %s mode (result=%u), retrying\033[0m",
                mode_name, static_cast<unsigned>(service_result_));
        }
    }

    // 同一时刻只允许一个 VehicleCommand 请求在途，避免控制周期内重复发送。
    if (!service_request_pending_) {
        request_vehicle_command_(request_type, command, param1, param2);
    }
    return false;
}
/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void PX4CtrlFSM::request_vehicle_command_(
    VehicleCommandRequest request_type, uint16_t command, double param1, double param2)
{
	// sun: 异步服务避免控制线程阻塞；完成标志由 response_callback_ 统一更新。
	if (!vehicle_command_client || !vehicle_command_client->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
            px4controlnode_.get_logger(), *px4controlnode_.get_clock(), 1000,
            "[px4ctrl] VehicleCommand service is not ready; mode request will retry");
        return;
    }
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->px4controlnode_.get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	service_request_pending_ = true;
    active_service_request_ = request_type;
	vehicle_command_client->async_send_request(
        request,
        [this, request_type](
            rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
            response_callback_(request_type, future);
        });
	// RCLCPP_INFO(this->px4controlnode_.get_logger(), "Command send");
}

void PX4CtrlFSM::response_callback_(
    VehicleCommandRequest request_type,
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) 
{
    // 服务回调触发时 future 已就绪；保存请求类型和结果，由 FSM 周期消费。
	auto reply = future.get()->reply;
	service_result_ = reply.result;
    switch (service_result_)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			// RCLCPP_INFO(this->px4controlnode_.get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command reply unknown");
			break;
		}
    if (active_service_request_ == request_type) {
        active_service_request_ = VehicleCommandRequest::NONE;
        service_request_pending_ = false;
    }
    completed_service_request_ = request_type;
    service_done = true;
}
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void PX4CtrlFSM::publish_offboard_control_mode_()
{
	// sun: 本工程自行完成控制和分配，因此声明 direct_actuator，其余 PX4 控制层全部关闭。
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = true;
	msg.timestamp = this->px4controlnode_.get_clock()->now().nanoseconds() / 1000; // 微秒
	offboard_control_mode_publisher->publish(msg);
}

void PX4CtrlFSM::publish_rates_thrust_setpoint()
{
    // sun: 自定义消息保留 ENU/FLU 角速度符号，底层节点负责最终转换和电机混控。
    ratectrl_msgs::msg::RatesThrustSetpoint msg{};
    msg.bodyrates[0] = control_sp_.bodyrates[0];
    msg.bodyrates[1] = control_sp_.bodyrates[1];
    msg.bodyrates[2] = control_sp_.bodyrates[2];
    if(std::isnan(msg.bodyrates[0]))
    {
        std::cout << "msg.bodyrates contains NAN!" << std::endl;
    }
    msg.thrust = control_sp_.thrust;
    msg.rate_dot_ref_valid = control_sp_.rate_dot_ref_valid;
    msg.rate_dot_ref[0] = control_sp_.rate_dot_ref[0];
    msg.rate_dot_ref[1] = control_sp_.rate_dot_ref[1];
    msg.rate_dot_ref[2] = control_sp_.rate_dot_ref[2];
	msg.timestamp = this->px4controlnode_.get_clock()->now().nanoseconds() / 1000;
	rates_thrust_setpoint_publisher->publish(msg);
}

// One native reference window and separate mode metadata for every controller.
void PX4CtrlFSM::calculate_control_()
{
    const double now = px4controlnode_.get_clock()->now().seconds();
#if PX4CTRL_PRIMARY_CONTROLLER == 0
    const int horizon = 0;
    const double prediction_dt = 1.0 / std::max(1.0, px4controlnode_.param.ctrl_freq_max);
#else
    const int horizon = controller.options().horizon;
    const double prediction_dt = controller.options().prediction_dt;
#endif
    const auto make_reference = [&]() {
        if (ref_.fsm_state == FSM_STATE(manual)) {
            return px4ctrl::ReferenceWindow{now, prediction_dt, {}};
        }
        if (trajectory_reference_.active()) {
            return trajectory_reference_.sample(now, horizon, prediction_dt);
        }
        px4ctrl::ReferencePoint point;
        point.position = ref_.p; point.velocity = ref_.v; point.acceleration = ref_.a;
        point.jerk = ref_.j; point.snap = ref_.s;
        point.yaw = get_yaw_from_quaternion(ref_.q);
        point.yaw_rate = ref_.yaw_rate; point.yaw_acceleration = ref_.yaw_accel;
        return px4ctrl::extrapolateFlatReference(
            point, now, horizon, prediction_dt, px4controlnode_.param.gra);
    };
    px4ctrl::ControlModeReference mode;
    mode.fsm_state = ref_.fsm_state; mode.attitude = ref_.q;
    mode.yaw_rate = ref_.yaw_rate; mode.throttle = ref_.throttle;
    mode.position_valid = ref_.flag_valid_p;
    mode.velocity_valid = ref_.flag_valid_v;
    mode.acceleration_valid = ref_.flag_valid_a;
    const auto calculate = [&](const px4ctrl::ReferenceWindow &window) {
        debug_msg = controller.calculateControl(window, mode, pose_data, att_data,
            sens_data, now, dt_, control_sp_, px4controlnode_.param);
    };
    try {
        calculate(make_reference());
    } catch (const std::invalid_argument &error) {
        // Never execute a stale preview after a rejected reference. Recover to
        // current-position hover, keeping the normal mode/Offboard lifecycle.
        RCLCPP_ERROR(px4controlnode_.get_logger(), "Invalid control reference: %s", error.what());
        trajectory_reference_.clear();
        reset_controller_();
        record_position();
        set_hover_ref();
        set_next_state(&fsm_, FSM_STATE(auto_hover));
        mode.fsm_state = ref_.fsm_state;
        mode.position_valid = mode.velocity_valid = mode.acceleration_valid = true;
        calculate(make_reference());
    }
#if PX4CTRL_PRIMARY_CONTROLLER == 2
    const auto &d = controller.diagnostics();
    if (d.fallback) {
        RCLCPP_WARN_THROTTLE(px4controlnode_.get_logger(), *px4controlnode_.get_clock(),
            1000, "[acados] Feedback fallback: status=%d, failures=%d, iter=%d, time=%.3f ms",
            d.status, d.consecutive_failures, d.iterations, d.solve_time_ms);
    }
#elif PX4CTRL_PRIMARY_CONTROLLER == 1
    const auto &d=controller.diagnostics();
    if (d.fallback) {
        RCLCPP_WARN_THROTTLE(px4controlnode_.get_logger(), *px4controlnode_.get_clock(), 1000,
            "[OMMPC] Feedback fallback: status=%d, failures=%d, cycle=%.3f ms",
            d.status,d.consecutive_failures,d.cycle_time_ms);
    }
#endif
}

void PX4CtrlFSM::reset_controller_()
{
#if PX4CTRL_PRIMARY_CONTROLLER == 2
    controller.reset();
#else
    controller.resetControlParams();
#endif
}
