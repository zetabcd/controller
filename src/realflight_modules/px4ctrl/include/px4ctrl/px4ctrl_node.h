#ifndef __PX4CTRL_NODE_H
#define __PX4CTRL_NODE_H

// sun: 顶层 ROS 2 节点负责参数装载、话题连接、节点握手和固定频率调度；
// sun: 飞行模式和控制算法分别封装在 PX4CtrlFSM 与所选顶层控制器中；实际主控制器
// sun: 只由 px4ctrlfsm.h 的 PX4CTRL_PRIMARY_CONTROLLER 当前值决定。
#include <stdint.h>
#include <chrono>
#include <iostream>

// #include <px4_ros_com/frame_transforms.h>
#include <px4ctrl/px4ctrlfsm.h> 
#include <px4ctrl/px4ctrlparam.h>
#include <string>
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class PX4ControlNode : public rclcpp::Node
{
public:
	PX4ControlNode(std::string name);
	PX4CtrlFSM fsm;
	Parameter_t param;
	// sun: param_init 保存启动时参数，运行中参数异常或需要复位时可作为可信基线。
	Parameter_t param_init;
	void init_param();
	void node_handshake_check(const std::string &server_node_name, const std::string &client_node_name);
	void config_from_ros_handle();
	void timer_callback();

private:
	rclcpp::TimerBase::SharedPtr main_timer_;
    // sun: 握手服务只利用服务调用是否成功判断对端在线，请求和响应均无业务负载。
    void handshake_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response){
        (void)response; // 消除未使用警告
        // RCLCPP_INFO(this->get_logger(), "收到握手请求，确认在线！");
    }
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr handshake_server_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr handshake_client_;
	rclcpp::TimerBase::SharedPtr timer_;
	// sun: 仿真使用自定义 JoyStick，实机使用 PX4 ManualControlSetpoint，其他反馈接口保持一致。
#ifdef SIMULATION
	rclcpp::Subscription<joy_msgs::msg::JoyStick>::SharedPtr joystick_subscription_;
#else
	rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_subscription_;
#endif
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_position_subscription_;
};

#endif
