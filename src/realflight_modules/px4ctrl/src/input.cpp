#include <px4ctrl/input.h>
#include <px4ctrl/px4ctrl_node.h>
// #include <px4ctrl/frame_transforms.h>
#include <uav_utils/geometry_utils.h>
#include <cmath>
#include <iostream>

// sun: 输入层只负责“消息 -> 内部物理量”转换，不在回调中执行控制计算，以缩短回调时间。

#define _FExpo(x) (x*x*x + (1-x*x)*x*0.7)

std::ostream& operator <<(std::ostream& os, const GEARS& gear) {
    switch (gear) {
        case GEARS::DOWN: os << "DOWN"; break;
        case GEARS::MID: os << "MID"; break;
        case GEARS::UP: os << "UP"; break;
        default: os << "Unknown"; break;
    }
    return os;
}

double inline _FThrottle(double x, double h)
{
    // sun: 分式变换将遥控中点移动到悬停油门 h，并保持输入端点 -1/1 不变。
    double m = -1+2*h;
    double p = (-m+x) / (-m*x+1);
    double f = (p*p*p + (1-p*p)*p*0.3);
    double re = (m+f) / (m*f+1);
    return re;
} 
double inline _FYaw(double x)
{
    // sun: 偏航曲线在零点附近降低灵敏度、端点保持满量程，便于精细航向操纵。
    double a = -0.7; // [-1,0] 0:y=x
    double re;
    if (x >= 0)
        re = x*(1-a)/(-a*(2*x-1)+1);
    else
        re = x*(1-a)/(a*(2*x+1)+1);
    return re;
} 
// using namespace frame_transforms;
// using namespace frame_transforms::utils::quaternion;
using namespace uav_utils;

RC_Data_t::RC_Data_t(PX4ControlNode& px4controlnode) : px4controlnode_(px4controlnode)
{
    // sun: 初始化为低风险开关组合；hover_percentage 不可为零，否则油门映射会退化。
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = 0ull;
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    throttle = 0.0f;
    aux1 = GEARS::DOWN;
    aux2 = GEARS::DOWN;
    aux3 = GEARS::DOWN;
    aux4 = GEARS::UP;
    aux5 = GEARS::DOWN;
    aux6 = GEARS::DOWN;
    sticks_moving = false;
    hover_percentage = 0.5; // 不能为0

    aux1_changed = false;
    aux2_changed = false;
    aux3_changed = false;
    aux4_changed = false;
    aux5_changed = false;
    aux6_changed = false;

    aux1_last_ = aux1;
    aux2_last_ = aux2;
    aux3_last_ = aux3;
    aux4_last_ = aux4;
    aux5_last_ = aux5;
    aux6_last_ = aux6;

    aux2_has_downed = false;
}
#ifdef SIMULATION
void RC_Data_t::feed(const joy_msgs::msg::JoyStick::UniquePtr msg)
{
    // sun: 仿真摇杆与实机遥控共用整形函数，保证切换运行环境后手感和状态机阈值一致。
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = msg->timestamp;
    roll = _FExpo(msg->roll);
    pitch = _FExpo(msg->pitch);
    yaw = _FYaw(msg->yaw);
    throttle = _FThrottle(msg->throttle,hover_percentage);
    // sun: 连续辅助通道乘 2 后饱和、截断为 -1/0/1，对应三段开关的三个档位。
    // (-1.5,-0.5] is DOWN
    // (-0.5,0.5] is MID
    // (0.5,1.5] is UP 
    aux1 = static_cast<GEARS>(SAT_1(msg->aux1 * 2.0));
    aux2 = static_cast<GEARS>(SAT_1(msg->aux2 * 2.0));
    aux3 = static_cast<GEARS>(SAT_1(msg->aux3 * 2.0));
    aux4 = GEARS::DOWN;

    aux1_changed = (aux1 != aux1_last_);
    aux2_changed = (aux2 != aux2_last_);
    aux3_changed = (aux3 != aux3_last_);
    aux4_changed = (aux4 != aux4_last_);

    aux1_last_ = aux1;
    aux2_last_ = aux2;
    aux3_last_ = aux3;
    aux4_last_ = aux4;

    // sun: aux2 必须实际经过 DOWN 后才接受后续 UP，防止启动时开关已在高档而误触发自动模式。
    //DOWN=-1, MID=0, UP=1,
    if (aux2 == GEARS::DOWN)
        aux2_has_downed = true;
    else if (aux2 == GEARS::UP && aux2_has_downed)
        aux2_has_downed = false;

    // std::cout<<"aux1:"<<aux1<<"         aux2:"<<aux2<<std::endl;
}
#else

void RC_Data_t::feed(const px4_msgs::msg::ManualControlSetpoint::UniquePtr msg)
{
    // sun: 实机消息已由 PX4 完成通道归一化；此处仍保留滚转、俯仰和偏航手感整形。
    if (!msg->valid || !std::isfinite(msg->roll) || !std::isfinite(msg->pitch) ||
        !std::isfinite(msg->yaw) || !std::isfinite(msg->throttle)) {
        return;
    }
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = msg->timestamp;
    roll = _FExpo(msg->roll);
    pitch = _FExpo(msg->pitch);
    yaw = _FYaw(msg->yaw);
    // throttle = _FThrottle(msg->throttle, hover_percentage); 
    throttle = msg->throttle;

    // (-1.5,-0.5] is DOWN
    // (-0.5,0.5] is MID
    // (0.5,1.5] is UP 
    // 未映射辅助通道可能为 NaN；此时保持上一档，避免制造虚假切换边沿。
    const auto decode_gear = [](float value, GEARS previous) {
        return std::isfinite(value) ?
            static_cast<GEARS>(SAT_1(value * 2.0)) : previous;
    };
    aux1 = decode_gear(msg->aux1, aux1_last_);
    aux2 = decode_gear(msg->aux2, aux2_last_);
    aux3 = decode_gear(msg->aux3, aux3_last_);
    aux4 = decode_gear(msg->aux4, aux4_last_);
    aux5 = decode_gear(msg->aux5, aux5_last_);
    aux6 = decode_gear(msg->aux6, aux6_last_);
    sticks_moving = msg->sticks_moving;

    aux1_changed = aux1 != aux1_last_;
    aux2_changed = aux2 != aux2_last_;
    aux3_changed = aux3 != aux3_last_;
    aux4_changed = aux4 != aux4_last_;
    aux5_changed = aux5_initialized_ && aux5 != aux5_last_;
    if (std::isfinite(msg->aux5)) {
        if (aux5_changed) ++aux5_change_count;
        aux5_initialized_ = true;
    }
    aux6_changed = aux6 != aux6_last_;
    aux1_last_ = aux1;
    aux2_last_ = aux2;
    aux3_last_ = aux3;
    aux4_last_ = aux4;
    aux5_last_ = aux5;
    aux6_last_ = aux6;

    if (aux2 == GEARS::DOWN) {
        aux2_has_downed = true;
    } else if (aux2 == GEARS::UP && aux2_has_downed) {
        aux2_has_downed = false;
    }

    // rclcpp::Time now = rcv_stamp;
    // static rclcpp::Time last_clear_count_time = px4controlnode_.get_clock()->now();
    // if ((now - last_clear_count_time).seconds() > 0.2)  //进入这里时，one_min_count表示1s内通过spinonce()进来了几次
    // {
    //     printf("throttle:%f\n",throttle);
    //     last_clear_count_time = now;
    // }
}
#endif


States_Data_t::States_Data_t(PX4ControlNode& px4controlnode) : px4controlnode_(px4controlnode)
{
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = 0ull;
    nav_state = 0u; // MANUAL
    arming_state = 1u; // DISARMED
    pre_flight_checks_pass = false;
}

void States_Data_t::feed(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
{
    // sun: 使用本地接收时间判断新鲜度，PX4 时间戳只用于记录和跨节点对齐。
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = msg->timestamp;
    nav_state = msg->nav_state;
    arming_state = msg->arming_state;
    pre_flight_checks_pass = msg->pre_flight_checks_pass;
}


Sensor_Data_t::Sensor_Data_t(PX4ControlNode& px4controlnode) : px4controlnode_(px4controlnode)
{
    rcv_stamp = px4controlnode_.get_clock()->now();
#if PX4CTRL_USE_FILTERED_IMU
    gyro_rcv_stamp_ = rcv_stamp;
    accel_rcv_stamp_ = rcv_stamp;
#endif
    timestamp = 0ull;
    w.setZero();
    a.setZero();
    a_nog.setZero();
}

#if PX4CTRL_USE_FILTERED_IMU
void Sensor_Data_t::feed_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg)
{
    gyro_rcv_stamp_ = px4controlnode_.get_clock()->now();
    gyro_received_ = true;
    // 兼容 timestamp 沿用陀螺消息的 timestamp；接收新鲜度分别检查两个话题。
    timestamp = msg->timestamp;
    rcv_stamp = std::min(gyro_rcv_stamp_, accel_rcv_stamp_);
    w << msg->xyz[0], -msg->xyz[1], -msg->xyz[2]; // FRD -> FLU
}

void Sensor_Data_t::feed_acceleration(const px4_msgs::msg::VehicleAcceleration::UniquePtr msg)
{
    accel_rcv_stamp_ = px4controlnode_.get_clock()->now();
    accel_received_ = true;
    rcv_stamp = std::min(gyro_rcv_stamp_, accel_rcv_stamp_);
    // xyz 仍为机体系比力；这里只做 FRD -> FLU，不重复补偿重力。
    a << msg->xyz[0], -msg->xyz[1], -msg->xyz[2];
    a_nog = a;
}

bool Sensor_Data_t::filtered_imu_is_received(const rclcpp::Time &now) const
{
    if (!gyro_received_ || !accel_received_ || !w.allFinite() || !a.allFinite()) {
        return false;
    }
    const double gyro_age = (now - gyro_rcv_stamp_).seconds();
    const double accel_age = (now - accel_rcv_stamp_).seconds();
    return gyro_age >= 0.0 && gyro_age < 0.5 && accel_age >= 0.0 && accel_age < 0.5;
}
#else
void Sensor_Data_t::feed(const px4_msgs::msg::SensorCombined::UniquePtr msg)
{
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = msg->timestamp;
    // SensorCombined 是 FRD 机体系；这里只转换成 FLU，世界系转换在控制器中结合姿态完成。
    w << msg->gyro_rad[0], -msg->gyro_rad[1], -msg->gyro_rad[2];
    a << msg->accelerometer_m_s2[0], -msg->accelerometer_m_s2[1], -msg->accelerometer_m_s2[2];
    a_nog = a;
}
#endif

Attitude_Data_t::Attitude_Data_t(PX4ControlNode& px4controlnode) : px4controlnode_(px4controlnode)
{
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = 0ull;
    q.setIdentity();
}

void Attitude_Data_t::feed(const px4_msgs::msg::VehicleAttitude::UniquePtr msg)
{
    rclcpp::Time now = px4controlnode_.get_clock()->now();
    rcv_stamp = now;
    timestamp = msg->timestamp;
    Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    // sun: 四元数按 (w,x,y,z) 构造；翻转 y、z 分量与 FRD/NED -> FLU/ENU 约定一致。
    q = Eigen::Quaterniond(q_ned.w(),q_ned.x(),-q_ned.y(),-q_ned.z());
    
    // Eigen::Matrix3d R_ned = q_ned.toRotationMatrix();
    // Eigen::AngleAxisd rotationX(M_PI, Eigen::Vector3d::UnitX()); // 旋转45度
    // Eigen::Matrix3d R = rotationX.toRotationMatrix();
    // Eigen::Matrix3d R_enu = R* R_ned;
    // Eigen::Vector3d ypr = R_enu.eulerAngles(2, 1, 0); // ZYX顺序
    // q = ypr_to_quaternion(Eigen::Vector3d(0.0, 0.0, M_PI)) * q_ned;
    // Eigen::Vector3d ypr = quaternion_to_ypr(q);
    // printf("rpy:%.2f,%.2f,%.2f\n",ypr[2], ypr[1], ypr[0]);

    // sun: 频率检查采用一秒计数，只告警而不丢弃数据，也不改变状态机的新鲜度判据。
    static int one_min_count = 9999;
    static rclcpp::Time last_clear_count_time = px4controlnode_.get_clock()->now();
    if ((now - last_clear_count_time).seconds() > 1.0)  //进入这里时，one_min_count表示1s内通过spinonce()进来了几次
    {
        if (one_min_count < 100)
        {
            RCLCPP_INFO(px4controlnode_.get_logger(),"\033[33mAttitude frequency seems lower than 100Hz, which is too low!\033[0m");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

LocalPose_Data_t::LocalPose_Data_t(PX4ControlNode& px4controlnode) : px4controlnode_(px4controlnode)
{
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = 0ull;
    recv_new_msg = false;
}

void LocalPose_Data_t::feed(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
{
    rclcpp::Time now = px4controlnode_.get_clock()->now();
    rcv_stamp = now;
    timestamp = msg->timestamp;
    recv_new_msg = true;

    // sun: PX4 局部位置为 NED，本控制器内部沿用 x 前、y 左、z 上的 ENU 符号。
    p << msg->x, -msg->y, -msg->z;
    v << msg->vx, -msg->vy, -msg->vz;
    xy_valid = msg->xy_valid;
    z_valid = msg->z_valid;
    v_xy_valid = msg->v_xy_valid;
    v_z_valid = msg->v_z_valid;
    // std::cout<<"----------------------------------------"<<std::endl;

    // check the frequency
    static int one_min_count = 9999;
    static rclcpp::Time last_clear_count_time = px4controlnode_.get_clock()->now();
    if ((now - last_clear_count_time).seconds() > 1.0)  //进入这里时，one_min_count表示1s内通过spinonce()进来了几次
    {
        if (one_min_count < 20)
        {
            RCLCPP_INFO(px4controlnode_.get_logger(),"\033[33mLocal Position frequency seems lower than 20Hz, which is too low!\033[0m");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

Battery_Data_t::Battery_Data_t(PX4ControlNode& px4controlnode) : px4controlnode_(px4controlnode)
{
    rcv_stamp = px4controlnode_.get_clock()->now();
    timestamp = 0ull;
}

void Battery_Data_t::feed(const px4_msgs::msg::BatteryStatus::UniquePtr msg)
{
    rclcpp::Time now = px4controlnode_.get_clock()->now();
    rcv_stamp = now;
    timestamp = msg->timestamp;
    voltage_filtered_v = msg->voltage_v;
    current_filtered_a = msg->current_a;
    remaining = msg->remaining;
    time_remaining_s = msg->time_remaining_s;

#ifndef SIMULATION
    // sun: 实机下正常电量每 30 秒提示一次；低于 5% 时提高到每秒告警，避免刷屏。
    static rclcpp::Time last_print_t = px4controlnode_.get_clock()->now();
    if (remaining > 0.05)
    {
        if ((now - last_print_t).seconds() > 30)
        {
            RCLCPP_INFO(px4controlnode_.get_logger(),"\033[33m[px4ctrl] Voltage=%.3f, percentage=%.3f\033[0m", voltage_filtered_v, remaining);
            last_print_t = now;
        }
    }
    else
    {
        if ((now - last_print_t).seconds() > 1)
        {
            RCLCPP_INFO(px4controlnode_.get_logger(),"\033[33m[px4ctrl] Dangerous! Voltage=%.3f, percentage=%.3f\033[0m", voltage_filtered_v, remaining);
            last_print_t = now;
        }
    }
#endif
}
