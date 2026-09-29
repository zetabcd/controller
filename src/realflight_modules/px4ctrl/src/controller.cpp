
#include <cmath>
#include <cstddef>
#include <iostream>
#include <px4ctrl/controller.h>
#include <px4ctrl/px4ctrl_node.h>
#include <uav_utils/geometry_utils.h>
#include <uav_utils/other_utils.h>
#include <px4ctrl/px4ctrlparam.h>
#include <px4ctrl/motor_calculate.h>

// sun: 该实现采用“位置 P -> 速度 PID -> 期望合力 -> 姿态 P”的串级结构，
// sun: 输出期望角速度和总推力，底层角速度节点再完成力矩控制与电机分配。

// using namespace frame_transforms::utils::quaternion;
using namespace uav_utils;

QuadControl::QuadControl(PX4ControlNode& px4controlnode) : px4controlnode_(px4controlnode)
{
    // sun: 积分器和饱和标志必须显式初始化，避免 Eigen 固定尺寸对象保留未定义值。
    vel_int_ = Eigen::Vector3d::Zero();
    ome_int_ = Eigen::Vector3d::Zero();
    saturation_positive_ = {false, false, false};
    saturation_negative_ = {false, false, false};
}

void QuadControl::resetControlParams()
{
    // sun: 在模式切换时清空有记忆的控制状态，防止旧模式误差在新模式中继续累积。
    vel_int_ = Eigen::Vector3d::Zero();
    ome_int_ = Eigen::Vector3d::Zero();
    saturation_positive_ = {false, false, false};
    saturation_negative_ = {false, false, false};

    reset_filters();
}

px4debug_msgs::msg::Px4ctrlDebug QuadControl::calculateControl(
    const px4ctrl::ReferenceWindow &window, const Ref_State_t &mode,
    const LocalPose_Data_t &pose, const Attitude_Data_t &att,
    const Sensor_Data_t &sens, const double &dt,
    Control_Setpoint_t &output, const Parameter_t &param)
{
    if (mode.fsm_state == FSM_STATE(manual)) {
        return calculateControl(mode, pose, att, sens, dt, output, param);
    }
    px4ctrl::validateReferenceWindow(window, 0, window.dt);
    const auto &point = window.points.front();
    Ref_State_t ref = mode;
    ref.p = point.position; ref.v = point.velocity; ref.a = point.acceleration;
    ref.j = point.jerk; ref.s = point.snap;
    ref.q = yaw_to_quaternion(point.yaw);
    ref.yaw_rate = point.yaw_rate; ref.yaw_accel = point.yaw_acceleration;
    ref.feedforward_attitude = point.attitude;
    ref.body_rate_ff = point.body_rate;
    ref.body_acceleration_ff = point.angular_acceleration_valid ?
        point.body_acceleration : Eigen::Vector3d::Zero();
    ref.explicit_feedforward = true;
    ref.acceleration_feedforward_valid = point.angular_acceleration_valid;
    return calculateControl(ref, pose, att, sens, dt, output, param);
}

px4debug_msgs::msg::Px4ctrlDebug QuadControl::calculateControl(
    const Ref_State_t &ref, 
    const LocalPose_Data_t &pose, 
    const Attitude_Data_t &att,
    const Sensor_Data_t &sens, 
    const double &dt,
    Control_Setpoint_t &control_sp,  
    const Parameter_t &param)
{
    /*
     * 输入/输出约定：
     *   ref        轨迹给定（p/v/a/jerk/snap、姿态和偏航导数）。
     *   pose       位置与速度反馈，p/v 均在 ENU 惯性系 I 中。
     *   att, sens  姿态反馈以及 FLU 机体系 B 中的角速度/比力。
     *   dt         本次积分使用的控制周期，单位 s。
     *   control_sp 函数的真正控制输出：期望机体角速度、总推力和角加速度前馈。
     *   返回值     仅是用于记录/画图的 debug 消息。
     *
     * 下面常见的 Eigen/C++ 操作：
     *   v.head(2)       取 v 的头 2 个元素，即 [x,y]；在赋值号左侧时只修改 x/y。
     *   v.tail<3>()     取最后 3 个元素；这里用于取四元数式向量的 x/y/z 虚部。
     *   M.col(i)        取矩阵第 i 列；旋转矩阵的三列就是机体 x/y/z 轴在 I 系的方向。
     *   .noalias()      告诉 Eigen 等号左右不共享存储，可少建临时矩阵；不改变数学结果。
     *   clip(x,lo,hi)   把标量/每个元素夹在 [lo,hi] 内。
     *   pow(x,2)        x 的平方；sqrt/sqrtf(x) 开平方，f 版本返回 float。
     *   dot/cross/norm  分别是点积、叉积和欧氏模；normalized() 返回同方向的单位向量。
     */

    // 将姿态四元数展开为旋转矩阵 Rbi：v_I = Rbi * v_B。
    // Rbi.transpose() 是其逆变换（旋转矩阵正交），v_B = Rbi_T * v_I。
    Eigen::Quaterniond q = att.q;
    Eigen::Matrix3d Rbi = q.toRotationMatrix(); // body(B) -> inertial(I)
    Eigen::Matrix3d Rbi_T = Rbi.transpose();
    Eigen::Vector3d vel_B = Rbi_T * pose.v; // ENU 世界系速度 -> FLU 机体系速度
    // col(0/1/2) 是当前机体三个单位轴在惯性系中的表示。
    Eigen::Vector3d x_B = Rbi.col(0);
    Eigen::Vector3d y_B = Rbi.col(1);
    Eigen::Vector3d z_B = Rbi.col(2);
    // FLU 机体系比力先由当前姿态旋转到 ENU，再加上 ENU 世界系重力，
    // 得到速度环 D 项所需的世界系平动加速度。
    Eigen::Vector3d acc_I =
        Rbi * sens.a + Eigen::Vector3d(0.0, 0.0, -param.gra);
    Eigen::Vector3d acc_I_lpf_;
    // sun: 加速度微分反馈先逐轴低通，抑制 IMU 噪声经 D 项放大后传到姿态设定值。
    // Eigen 的 << 是逗号式初始化/填充语法，不是 C++ 比特左移。
    acc_I_lpf_ << 
        lpf_acc_x_->filter(acc_I[0]),
        lpf_acc_y_->filter(acc_I[1]),
        lpf_acc_z_->filter(acc_I[2]);

    // sun: 当前未接入风速估计，因此默认空气相对地面静止；接入风场后应在此扣除机体系风速。
    Eigen::Vector3d vel_w_B = Eigen::Vector3d::Zero(); //  <TODO>
    Eigen::Vector3d vel_a_B = vel_B - vel_w_B;
    // 会被累加的向量以及默认机体轴先给确定初值，避免使用 Eigen 未初始化数据。
    double collective_thrust_des;
    Eigen::Vector3d vel_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_des = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_des;
    Eigen::Vector3d x_B_des = Eigen::Vector3d::UnitX(); 
    Eigen::Vector3d y_B_des = Eigen::Vector3d::UnitY(); 
    Eigen::Vector3d z_B_des = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d Rbi_des;
    Eigen::Vector3d rate_ref = Eigen::Vector3d::Zero();
    Eigen::Vector3d rate_dot_ref = Eigen::Vector3d::Zero();
    if (ref.fsm_state == FSM_STATE(manual))
    {
        // sun: 手动模式下油门经标定多项式换算为转速，总推力再由桨模型计算；
        // sun: 姿态参考直接取遥控生成的 ref.q，不启用位置/速度闭环。
        // 二次多项式是实验标定的“遥控油门 -> 电机角速度”映射。
        double speed_rad_des = param.motor.rc2speed_a * ref.throttle * ref.throttle + param.motor.rc2speed_b * ref.throttle + param.motor.rc2speed_c;
        // Array4d::Constant 创建 4 个元素都相同的数组，表示四个电机同转速。
        Eigen::Array4d motor_rad_des = Eigen::Array4d::Constant(speed_rad_des);
        Eigen::Array4d motor_thrust_des = get_thrust_from_speed(motor_rad_des,motor_rad_des, vel_a_B, param);
        collective_thrust_des = motor_thrust_des.sum();
        q_des = ref.q;
        rate_ref << 0.0, 0.0, ref.yaw_rate * Eigen::Vector3d::UnitZ().dot(z_B);
    }else{
        // asDiagonal() 把 [kx,ky,kz] 变成 diag(kx,ky,kz)。
        // Rbi*K*Rbi_T 把在机体轴上定义的三轴增益旋转到惯性系中使用。
        Eigen::Matrix3d gain_pos = Eigen::Vector3d(param.gain.gain_pos_x, param.gain.gain_pos_y, param.gain.gain_pos_z).asDiagonal();
        Eigen::Matrix3d gain_vel_p = Eigen::Vector3d(param.gain.gain_vel_p_x, param.gain.gain_vel_p_y, param.gain.gain_vel_p_z).asDiagonal();
        Eigen::Matrix3d gain_vel_i = Eigen::Vector3d(param.gain.gain_vel_i_x, param.gain.gain_vel_i_y, param.gain.gain_vel_i_z).asDiagonal();
        Eigen::Matrix3d gain_vel_d = Eigen::Vector3d(param.gain.gain_vel_d_x, param.gain.gain_vel_d_y, param.gain.gain_vel_d_z).asDiagonal();

        // 位置 P 环：v_correction = Kp_pos * (p_ref - p)。bool 标志参与乘法时
        // 会转成 0/1，因此 flag_valid_p=false 就会关掉位置反馈项。
        vel_des = Rbi * gain_pos * Rbi_T * (ref.p - pose.p) * ref.flag_valid_p;
        // head(2) 是 [x,y]。constrainXY(v0,v1,max) 优先保留位置环给出的 v0，
        // 再尽量加入轨迹前馈 v1，最终保证水平速度向量的模 <= lim_vel_horizontal。
        vel_des.head(2) = constrainXY(vel_des.head(2), ref.v.head(2), param.other.lim_vel_horizontal);
        // z() 是第 3 个元素的可读写引用。z 速度叠加前馈后单独做上升/下降限幅。
        vel_des.z() += ref.v.z();
        vel_des.z() = clip(vel_des.z(), -param.other.lim_vel_down, param.other.lim_vel_up);
        
        // 速度 PID 输出期望平动加速度。先把 z 轴已积累的 I 项限制在 +/-g，
        // 防止长时间误差使竖直积分无界增长。
        vel_int_.z() = clip(vel_int_.z(), -param.gra, param.gra);
        Eigen::Vector3d vel_err = vel_des - pose.v;
        // noalias() 只是 Eigen 的性能提示；下面四行数学上就是顺序累加：
        // acc_des = Kp_v*e_v + I_v - Kd_v*a_measured + a_feedforward。
        acc_des.noalias() += Rbi * gain_vel_p * Rbi_T * vel_err * ref.flag_valid_v;
        acc_des.noalias() += vel_int_ * ref.flag_valid_v;
        // 用 0 - acc_I_lpf_ 代替直接对速度误差差分，可避免对噪声较大的速度信号做数值微分。
        acc_des.noalias() += Rbi * gain_vel_d * Rbi_T * (Eigen::Vector3d::Zero() - acc_I_lpf_) * ref.flag_valid_v;
        // ref.a 是轨迹加速度前馈，不依赖误差；flag_valid_a=false 时关闭。
        acc_des.noalias() += ref.a * ref.flag_valid_a; 

        Eigen::Vector3d g(0,0,-param.gra);
        // sun: 简化机体气动力在 B 系计算后旋转回 I 系，期望合力满足
        // sun: F_des = m(a_des - g) - F_aero。
        // v[i] 与 v(i) 都是按下标取元素。前两轴是线性阻力，z 轴还含水平速度平方项。
        Eigen::Vector3d fa_B(-param.aero.kdx * vel_B[0], 
                            -param.aero.kdy * vel_B[1], 
                            -param.aero.kdz * vel_B[2] + param.aero.kh * (vel_B[0] * vel_B[0] + vel_B[1] * vel_B[1]));
        Eigen::Vector3d collective_thrust_des_vec = (acc_des - g) * param.uav.mass - Rbi * fa_B;//期望物理推力向量
        
        
        //----------------------------------------------------------------------------------------------------
        // sun: 竖直推力触碰边界且误差还会使输出继续饱和时，冻结该方向积分输入。
        if ((collective_thrust_des_vec.z() <= param.motor.u_min*4 && vel_err.z() >= 0.f) ||
            (collective_thrust_des_vec.z() >= param.motor.u_max*4 && vel_err.z() <= 0.f)) {
            vel_err.z() = 0.0;
        }

        // sun: 推力按“保留水平余量后优先满足竖直分量”的策略投影到总推力球内，
        // sun: 避免逐轴限幅破坏合力方向，并为姿态控制保留最低水平机动能力。
        const Eigen::Vector2d thrust_des_xy = collective_thrust_des_vec.head(2); // 拷贝限幅前的 [Fx,Fy]
        const double thrust_sp_xy_norm = thrust_des_xy.norm();
        // pow(x,2) 求 x^2。四个电机每个上限 u_max，因而总推力上限按 4*u_max 估计。
        const double thrust_max_squared = pow(param.motor.u_max*4,2);
        // 计算保留水平余量后，垂直推力的最大可用值
        const double allocated_horizontal_thrust = std::min(thrust_sp_xy_norm, param.other.lim_thr_xy_margin);
        const double thrust_z_max_squared = thrust_max_squared - pow(allocated_horizontal_thrust,2);
        // std::min 只限制 z 推力的上界；sqrt 把“推力平方余量”还原为推力。
        collective_thrust_des_vec.z() = std::min(collective_thrust_des_vec.z(), std::sqrt(thrust_z_max_squared));
        // 根据垂直推力，重新计算水平推力的最大可用值
        const double thrust_max_xy_squared = thrust_max_squared - pow(collective_thrust_des_vec.z(),2);
        double thrust_max_xy = 0.f;
        if (thrust_max_xy_squared > 0.f) {
            // sqrtf 是 float 版开平方；这里会再隐式转成 double 存入 thrust_max_xy。
            thrust_max_xy = sqrtf(thrust_max_xy_squared);
        }
        if (thrust_sp_xy_norm > thrust_max_xy) {
            // 先除以 norm 取得原水平推力的方向，再乘可用上限：只缩短，不改方向。
            collective_thrust_des_vec.head(2) = thrust_des_xy / thrust_sp_xy_norm * thrust_max_xy;
        }





        // sun: 水平采用跟踪式抗饱和：把受限前后的加速度差反馈到速度误差，
        // sun: 使积分器跟随实际可实现输出，解除饱和后能更快恢复。
        // 推力→加速度转换：实际水平推力能产生的加速度
        const Eigen::Vector2d acc_des_xy_produced = collective_thrust_des_vec.head(2) / param.uav.mass;//限幅后剩下的加速度
        const double arw_gain = 2. / param.gain.gain_pos_x; // ARW增益（和比例增益成反比）
        // 判断是否饱和：期望加速度 > 实际能产生的加速度 -> 饱和
        const Eigen::Vector2d acc_des_xy = acc_des.head(2);
        // dot(v,v) = ||v||^2，比较平方模可避免两次开方。?: 是 C++ 三元运算符：
        // 期望模更大时选可实现加速度，否则保留原期望加速度。
        const Eigen::Vector2d acc_limited_xy = (acc_des_xy.dot(acc_des_xy) > acc_des_xy_produced.dot(acc_des_xy_produced))
                        ? acc_des_xy_produced
                        : acc_des_xy;
        vel_err.head(2) = vel_err.head(2) - arw_gain * (acc_des_xy - acc_limited_xy);

        // 离散积分：I[k+1] = I[k] + Ki*e*dt。前面已对 e 做了竖直冻结和水平抗饱和修正。
        vel_int_ += gain_vel_i * vel_err * dt;

        // sun: 期望合力方向定义期望机体 z 轴，偏航角提供水平航向约束；二者正交化后
        // sun: 组成合法旋转矩阵。极小推力、倒置和水平推力情况均有退化保护。
        // norm() 将三维期望合力变成标量总推力，再由在线模型换算为底层推力信号。
        collective_thrust_des = collective_thrust_des_vec.norm();
        collective_thrust_des = computeDesiredCollectiveThrustSignal(collective_thrust_des, param.uav.mass);


        //-------------------------------------------------------------------------------------------------------
        double yaw_des = get_yaw_from_quaternion(ref.q);
        // 四旋翼总推力沿机体 +z 轴，所以期望合力的单位方向就是期望 z_B。
        z_B_des = collective_thrust_des_vec.normalized();
        if (z_B_des.dot(z_B_des) < FLT_EPSILON) {
            z_B_des.z() = 1.;
        }

        // y_C_des 是仅由期望偏航角确定的水平参考 y 轴。
        // cross() 做叉积，构造同时垂直于 y_C_des 和 z_B_des 的期望 x 轴。
        Eigen::Vector3d y_C_des(-sin(yaw_des), cos(yaw_des), 0);
        x_B_des = y_C_des.cross(z_B_des);
        // 倒立时保持机头朝前
        if (z_B_des.z() < 0.f) {
            x_B_des = -x_B_des;
        }
        if (std::abs(z_B_des.z()) < 0.000001) {
            // 所需的推力在 XY 平面，将 X 设置向下保证矩阵构建正确，但偏航分量失效
            x_B_des = -Eigen::Vector3d::UnitZ();
        }

        // normalize() 是原地单位化（修改对象本身）；normalized() 则返回单位化后的副本。
        // 依次将 x/y/z 轴作为列填入 Rbi_des，再转成期望四元数。
        x_B_des.normalize();
        y_B_des = z_B_des.cross(x_B_des);
        y_B_des.normalize();
        Rbi_des << x_B_des, y_B_des, z_B_des;
        Rbi_des << x_B_des, y_B_des, z_B_des;
        q_des = Eigen::Quaterniond(Rbi_des); 




        if (ref.explicit_feedforward) {
            const Eigen::Quaterniond reference_to_actual =
                att.q.normalized().conjugate() * ref.feedforward_attitude;
            rate_ref = reference_to_actual * ref.body_rate_ff;
            rate_dot_ref = reference_to_actual * ref.body_acceleration_ff;
        } else {
        // sun: jerk 和 snap 通过微分平坦关系映射为角速度、角加速度前馈；
        // sun: last_Td 使用上一周期推力，避免当前求导链路形成代数环。
        Eigen::Vector3d h_ome;
        Eigen::Vector3d h_alpha;
        static double last_Td = collective_thrust_des;
        // dot() 把 jerk/snap 投影到当前推力轴；cross() 表示角速度引起的方向变化。
        double diff_T = param.uav.mass * ref.j.dot(z_B);
        double diff2_T = param.uav.mass * ref.s.dot(z_B) + param.uav.mass * sens.w.cross(z_B).dot(ref.j);
        h_ome = (param.uav.mass * ref.j - diff_T * z_B) / last_Td;
        rate_ref << -h_ome.dot(y_B), h_ome.dot(x_B), ref.yaw_rate * Eigen::Vector3d::UnitZ().dot(z_B);
        h_alpha = param.uav.mass / last_Td * ref.s - (sens.w.cross(sens.w.cross(z_B)) + 
                                2 * diff_T / last_Td * sens.w.cross(z_B) + diff2_T / last_Td * z_B);
        rate_dot_ref << -h_alpha.dot(y_B), h_alpha.dot(x_B), ref.yaw_accel * Eigen::Vector3d::UnitZ().dot(z_B);
        last_Td = collective_thrust_des;
        }

    }


    // sun: 调参开关打开时以固定欧拉角覆盖正常轨迹姿态，便于单独验证姿态环响应。
    if (param.tuning.attitude_loop){ // 姿态环调参
        Eigen::Vector3d ypr_des(
            deg2rad(param.tuning.euler_des_z_deg), 
            deg2rad(param.tuning.euler_des_y_deg), 
            deg2rad(param.tuning.euler_des_x_deg));
        q_des = ypr_to_quaternion(ypr_des);
    }

    // 相对四元数 qe = q^{-1}*q_des，表示“从当前姿态转到期望姿态”的旋转。
    Eigen::Quaterniond qe = q.inverse() * q_des;
    // sun: 将姿态误差分解为倾转 qe_red 与偏航 qe_yaw 并分别施加增益，降低大倾角时
    // sun: 偏航误差对推力方向跟踪的干扰；sgn(qe.w()) 选择四元数最短旋转路径。
    Eigen::Vector4d qe_red = 1 / (std::sqrt(qe.w()*qe.w() + qe.z()*qe.z())) * 
        Eigen::Vector4d(qe.w()*qe.w() + qe.z()*qe.z(),
                          qe.w()*qe.x() - qe.y()*qe.z(),
                          qe.w()*qe.y() + qe.x()*qe.z(),
                          0.0);
    Eigen::Vector4d qe_yaw = 1 / (std::sqrt(qe.w()*qe.w() + qe.z()*qe.z())) * 
        Eigen::Vector4d(qe.w(),
                          0.0,
                          0.0,
                          qe.z()); 
    
    Eigen::Matrix3d gain_quat_red = Eigen::Vector3d(param.gain.gain_quat_x, param.gain.gain_quat_y, 0.0).asDiagonal();
    double gain_quat_yaw = param.gain.gain_quat_z;

    // tail<3>() 取 [x,y,z] 虚部。倾转误差只对 roll/pitch 施加增益，
    // 偏航误差单独乘 yaw 增益，最后叠加微分平坦给出的角速度前馈。
    Eigen::Vector3d rate_des = gain_quat_red * qe_red.tail<3>() + gain_quat_yaw * sgn(qe.w()) * qe_yaw.tail<3>() + rate_ref;

    if (param.tuning.angular_rate_loop){ // 角速度环调参
        // sun: 角速度调参模式直接覆盖姿态环输出，并用悬停推力保持高度附近的工作点。
        rate_des << 
            deg2rad(param.tuning.rate_des_x_deg),
            deg2rad(param.tuning.rate_des_y_deg),
            deg2rad(param.tuning.rate_des_z_deg);
        collective_thrust_des = param.uav.mass * param.gra;
    }

    // 将控制结果写入引用参数 control_sp，由下游角速度环/电机分配器消费。
    control_sp.bodyrates = rate_des;
    control_sp.thrust = collective_thrust_des;
    control_sp.rate_dot_ref = rate_dot_ref;
    control_sp.rate_dot_ref_valid = ref.fsm_state != FSM_STATE(manual) &&
        (!ref.explicit_feedforward || ref.acceleration_feedforward_valid);
    if (param.tuning.attitude_loop || param.tuning.angular_rate_loop) {
        control_sp.rate_dot_ref.setZero();
        control_sp.rate_dot_ref_valid = false;
    }

    // static auto last_print_time = px4controlnode_.get_clock()->now();
    // auto now_time = px4controlnode_.get_clock()->now();
    // if ((now_time-last_print_time).seconds() > 0.1)
    // {
    //     std::cout << "yaw_des:" << yaw_des << std::endl;
    //     // std::cout << "ome_des:" << ome_des.transpose() << std::endl;
    //     // std::cout << "ome_dot_des:" << ome_dot_des.transpose() << std::endl;
    //     // std::cout << "Td_and_taud:" << Td_and_taud.transpose() << std::endl;
    //     // std::cout << "motor_thrust_sol:" << motor_thrust_sol.transpose() << std::endl;
    //     // std::cout << "speed_rad_des:" << speed_rad_des_s.transpose() << std::endl;
    //     // std::cout << "ctrl_output.thro_setpoint:" << ctrl_output.thro_setpoint.transpose() << std::endl;
    //     last_print_time = now_time;
    // }
    
    // 

    // sun: 调试消息沿用 PX4/NED 的 y、z 符号约定，从内部 ENU 输出时需翻转相应轴。
    debug_msg_.ref_p_x = ref.p[0];
    debug_msg_.ref_p_y = -ref.p[1];
    debug_msg_.ref_p_z = -ref.p[2];

    debug_msg_.ref_v_x = ref.v[0];
    debug_msg_.ref_v_y = -ref.v[1];
    debug_msg_.ref_v_z = -ref.v[2];

    debug_msg_.ref_a_x = ref.a[0];
    debug_msg_.ref_a_y = -ref.a[1];
    debug_msg_.ref_a_z = -ref.a[2];

    debug_msg_.ref_rate_x = rate_ref[0];
    debug_msg_.ref_rate_y = -rate_ref[1];
    debug_msg_.ref_rate_z = -rate_ref[2];

    debug_msg_.ref_rate_dot_x = control_sp.rate_dot_ref[0];
    debug_msg_.ref_rate_dot_y = -control_sp.rate_dot_ref[1];
    debug_msg_.ref_rate_dot_z = -control_sp.rate_dot_ref[2];

    debug_msg_.des_a_x = acc_des[0];
    debug_msg_.des_a_y = -acc_des[1];
    debug_msg_.des_a_z = -acc_des[2];

    debug_msg_.des_q_w = q_des.w();
    debug_msg_.des_q_x = q_des.x();
    debug_msg_.des_q_y = -q_des.y();
    debug_msg_.des_q_z = -q_des.z();

    Eigen::Vector3d ypr_d = quaternion_to_ypr(q_des);
    debug_msg_.des_roll = ypr_d[2];
    debug_msg_.des_pitch = -ypr_d[1]; 
    debug_msg_.des_yaw = -ypr_d[0]; 
    // debug_msg_.des_ang_yaw = -ypr_d[0]; 

    debug_msg_.des_rate_x = rate_des[0];
    debug_msg_.des_rate_y = -rate_des[1];
    debug_msg_.des_rate_z = -rate_des[2];

    debug_msg_.des_thrust = collective_thrust_des;

    rclcpp::Time now = px4controlnode_.get_clock()->now();
    debug_msg_.timestamp = now.nanoseconds() / 1000;

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<rclcpp::Time, double>(now, collective_thrust_des));
    while (timed_thrust_.size() > 100)
    {
        timed_thrust_.pop();
    }
    debug_msg_.thr2acc = thr2acc_;

    return debug_msg_;

}

void QuadControl::init_filters(const Parameter_t &param)
{
    // sun: 滤波器采样频率与顶层控制频率一致，三个轴可按传感器噪声分别设截止频率。
    double fs = param.ctrl_freq_max;
    lpf_acc_x_ = std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_acc_x_cutoff_hz, fs);
    lpf_acc_y_ = std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_acc_y_cutoff_hz, fs);
    lpf_acc_z_ = std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_acc_z_cutoff_hz, fs);
}

void QuadControl::reset_filters()
{
    // sun: reset 只清内部历史而不重新分配对象，适合在模式切换的实时路径调用。
    lpf_acc_x_->reset();
    lpf_acc_y_->reset();
    lpf_acc_z_->reset();
}

/*
  compute throttle percentage 
*/
double 
QuadControl::computeDesiredCollectiveThrustSignal(const double &thrust_des, const double &uav_mass)
{
    // sun: 利用在线辨识的 thr2acc_ 将期望加速度换算为飞控推力信号；映射失效时
    // sun: 回退到牛顿制推力，避免 NaN 或负比例继续传播。
    double acc_z_des = thrust_des / uav_mass; // 期望的垂直加速度（不包含重力）
    double thrust(0.0);
    
    /* compute throttle, thr2acc has been estimated before */
    if (thr2acc_ >= 0 && std::isfinite(thr2acc_))
    {
        thrust = acc_z_des / thr2acc_;
    }else{
        thrust = thrust_des;
    }
    return thrust;
}

bool 
QuadControl::estimateThrustModel(const Eigen::Vector3d &est_a)
{
  // sun: 取约 35~45 ms 前发送的推力与当前加速度配对，以补偿电机、飞控和通信延迟。
  rclcpp::Time t_now = px4controlnode_.get_clock()->now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<rclcpp::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).seconds();
    if (time_passed > 0.045) // 45ms
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      return false;
    }

    // sun: 标量递推最小二乘只估计竖直通道比例，遗忘因子 rho2_ 允许模型缓慢跟踪
    // sun: 电池电压、载荷和桨效率变化，clip 则限制异常样本造成的单次漂移。
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    thr2acc_ = clip(thr2acc_, 0.2, 2.0); // 1.368
    P_ = (1 - K * thr) * P_ / rho2_;
    // printf("%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_, est_a(2));
    return true;
  }
  return false;
}

void 
QuadControl::resetThrustMapping(const Parameter_t &param)
{
    // sun: 初值采用单位推力对应 1/mass 的比例，P_ 较大表示启动阶段估计不确定度高。
    thr2acc_ = 1.0 / param.uav.mass;  
    P_ = 1e6;
}  
