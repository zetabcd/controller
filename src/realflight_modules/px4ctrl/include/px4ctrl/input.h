#ifndef __INPUT_H
#define __INPUT_H

// sun: 本文件将不同 PX4/仿真消息统一整理为控制器内部状态，并在接收入口完成
// sun: 坐标系、单位、遥控曲线和时间戳转换，使后续控制算法不依赖消息格式。

#ifndef PX4CTRL_SIMULATION
#define PX4CTRL_SIMULATION 1
#endif
#if PX4CTRL_SIMULATION
#define SIMULATION
#endif
// [NON-CONTROLLER MODE CHANGE]
// 该宏与 QuadControl/OmMpc/NMPC 算法无关。启用后 FSM 默认从 manual 开始，
// process() 会合成遥控器挡位，并按启动相对时间自动切换 AUTO_HOVER/CMD。
// 它会让“第一帧传感器是否早于一次性挡位边沿到达”影响仿真启动结果。
#if PX4CTRL_SIMULATION
#define USE_WITHOUT_RC
#endif

// IMU 数据源统一开关，外环和角速度内环共用；修改后重新编译 px4ctrl。
// 0：保留原 /fmu/out/sensor_combined 实现（当前 quadsim 也使用此话题）。
// 1：使用 PX4 已滤波的 vehicle_angular_velocity + vehicle_acceleration。
// 只切换输入源；角加速度仍由现有 TVR 计算，控制器原有滤波逻辑保持不变。
#ifndef PX4CTRL_USE_FILTERED_IMU
#define PX4CTRL_USE_FILTERED_IMU 0
#endif
#if PX4CTRL_USE_FILTERED_IMU != 0 && PX4CTRL_USE_FILTERED_IMU != 1
#error "PX4CTRL_USE_FILTERED_IMU must be 0 or 1"
#endif

#define SAT_1(x) (std::min(std::max(x, -1.0), 1.0)) //饱和函数

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#if PX4CTRL_USE_FILTERED_IMU
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_acceleration.hpp>
#endif
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <joy_msgs/msg/joy_stick.hpp>
#include <uav_utils/other_utils.h>

enum class GEARS : int8_t
{
    // sun: 三段开关值与数值 -1/0/1 一一对应，可直接用于有限状态机的模式选择。
    DOWN=-1, 
    MID=0, 
    UP=1,
};

std::ostream& operator <<(std::ostream& os, const GEARS& gear);

class PX4ControlNode;

class RC_Data_t
{
public:
    // sun: 四个主通道均归一化到 [-1, 1]；油门会经过悬停点重映射。
    double roll;
    double pitch;
    double yaw;
    double throttle; // [-1,1]
    GEARS aux1;
    GEARS aux2;
    GEARS aux3;
    GEARS aux4;
    GEARS aux5;
    GEARS aux6;
    bool sticks_moving;
    double hover_percentage;

    uint64_t timestamp;
    // sun: timestamp 来自 PX4/仿真端，rcv_stamp 是本 ROS 节点实际收到消息的时刻。
    rclcpp::Time rcv_stamp;

    GEARS aux1_last_;
    GEARS aux2_last_;
    GEARS aux3_last_;
    GEARS aux4_last_;
    GEARS aux5_last_;
    GEARS aux6_last_;

    bool aux1_changed;
    bool aux2_changed;
    bool aux3_changed;
    bool aux4_changed;
    bool aux5_changed;
    bool aux6_changed;
    // Monotonic count preserves multiple AUX5 edges received in one spin_some().
    uint64_t aux5_change_count{0};

    bool aux2_has_downed;

    // sun: feed() 同时更新当前开关值、边沿变化标志和上一帧状态。
    RC_Data_t(PX4ControlNode &);
#ifdef SIMULATION
    void feed(const joy_msgs::msg::JoyStick::UniquePtr msg);
#else
    void feed(const px4_msgs::msg::ManualControlSetpoint::UniquePtr msg);
#endif

private:
    PX4ControlNode& px4controlnode_;
    bool aux5_initialized_{false};

};

class States_Data_t
{
public:
    // sun: 保存 PX4 导航、解锁和起飞前检查状态，供状态机决定能否切换模式或解锁。
    uint8_t nav_state;
    uint8_t arming_state; //DISARMED = 1; ARMED = 2
    bool pre_flight_checks_pass;

    uint64_t timestamp;
    rclcpp::Time rcv_stamp;
    
    States_Data_t(PX4ControlNode& px4controlnode);
    void feed(const px4_msgs::msg::VehicleStatus::UniquePtr msg);
private:
    PX4ControlNode& px4controlnode_;
};

class Sensor_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d w; // [rad/s] body FLU
    // 加速度计与陀螺仪都在 FLU 机体系；世界系加速度需要结合姿态计算。
    Eigen::Vector3d a; // [m/s^2] body FLU specific force
    Eigen::Vector3d a_nog; // 兼容字段，仍保存 FLU 比力

    uint64_t timestamp;
    rclcpp::Time rcv_stamp;

    Sensor_Data_t(PX4ControlNode& px4controlnode);
#if PX4CTRL_USE_FILTERED_IMU
    void feed_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg);
    void feed_acceleration(const px4_msgs::msg::VehicleAcceleration::UniquePtr msg);
    bool filtered_imu_is_received(const rclcpp::Time &now) const;
#else
    void feed(const px4_msgs::msg::SensorCombined::UniquePtr msg);
#endif
private:
    PX4ControlNode& px4controlnode_;
#if PX4CTRL_USE_FILTERED_IMU
    // 两个话题独立更新；一方持续到达不能掩盖另一方未收到或超时。
    rclcpp::Time gyro_rcv_stamp_;
    rclcpp::Time accel_rcv_stamp_;
    bool gyro_received_{false};
    bool accel_received_{false};
#endif
};

class Attitude_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // sun: q 对应从机体系 B 到 ENU 惯性系 I 的主动旋转，Rbi = q.toRotationMatrix()。
    Eigen::Quaterniond q; // q_e^b --> R_b^e

    uint64_t timestamp;
    rclcpp::Time rcv_stamp;

    Attitude_Data_t(PX4ControlNode& px4controlnode);
    void feed(const px4_msgs::msg::VehicleAttitude::UniquePtr msg);
private:
    PX4ControlNode& px4controlnode_;
};

class LocalPose_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p; // [m] ENU
    Eigen::Vector3d v; // [m/s] ENU
    bool xy_valid;
    bool z_valid;
    bool v_xy_valid;
    bool v_z_valid;

    uint64_t timestamp;
    rclcpp::Time rcv_stamp;
    bool recv_new_msg;
    // sun: recv_new_msg 是一次性消费标志，用于保证控制循环只处理新到达的位置样本。

    LocalPose_Data_t(PX4ControlNode& px4controlnode);
    void feed(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);
private:
    PX4ControlNode& px4controlnode_;
};

class Battery_Data_t
{
public:
    // sun: 电池量只做状态监视和调试发布，不参与当前控制律的推力补偿。
    double voltage_filtered_v;
    double current_filtered_a;
    double remaining;
    double time_remaining_s;

    uint64_t timestamp;
    rclcpp::Time rcv_stamp;

    Battery_Data_t(PX4ControlNode& px4controlnode);
    void feed(const px4_msgs::msg::BatteryStatus::UniquePtr msg);
private:
    PX4ControlNode& px4controlnode_;
};

#endif
