#ifndef PX4CTRL_OMMPC_H_
#define PX4CTRL_OMMPC_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

#include <px4ctrl/controller.h>
#include <px4ctrl/omtraj.h>

#include <string>
#include <vector>

class PX4ControlNode;

/**
 * @brief MPC 预测时域中单个节点的名义状态与名义输入。
 *
 * 数学上保存
 *   x_k^d=(p_k^d,v_k^d,R_k^d),
 *   u_k^d=(a_{T,k}^d,omega_k^d)。
 * 在线 QP 优化的不是这些绝对量，而是相对于它们的输入修正
 * delta_u_k=(delta_a_T,delta_omega)。
 */
struct OmMpcReference
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 世界坐标系中的期望位置 p^d [m]。
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  // 世界坐标系中的期望速度 v^d [m/s]。
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  // 从机体系到世界系的期望姿态 R^d，以单位四元数保存。
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  // 质量归一化总拉力 a_T^d=F_T/m [m/s^2]，沿机体 +Z 轴。
  double thrust_acceleration{9.805};
  // 机体系期望角速度 omega^d=[p,q,r] [rad/s]。
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
};

/**
 * @brief 在线流形 MPC 的配置。
 *
 * 误差状态顺序固定为
 *   delta_x=[delta_p(3),delta_v(3),delta_theta(3)]，
 * 输入顺序固定为
 *   delta_u=[delta_a_T,delta_omega_x,delta_omega_y,delta_omega_z]。
 */
struct OmMpcOptions
{
  // 预测区间数 H；参考节点数量为 H+1，QP 输入块数量为 H。
  int horizon{8};
  // 调用者没有提供有效控制周期时使用的预测步长 [s]。
  double prediction_dt{0.01};
  // 真实质量归一化总拉力上下界 [m/s^2]。
  double thrust_acceleration_min{0.5};
  double thrust_acceleration_max{4.0 * 9.805};
  // 三轴真实机体系角速度绝对值上界 [rad/s]。
  Eigen::Vector3d body_rate_max{Eigen::Vector3d::Constant(14.0)};
  // Q=diag(q_p,q_v,q_R)，依次惩罚位置、速度和 SO(3) 局部姿态误差。
  Eigen::Matrix<double, 9, 1> state_weight{
    (Eigen::Matrix<double, 9, 1>() <<
      15000.0, 15000.0, 15000.0, 40.0, 40.0, 40.0, 80.0, 80.0, 80.0).finished()};
  // R=diag(r_T,r_omega)，惩罚相对离线轨迹前馈输入的修正量。
  Eigen::Matrix<double, 4, 1> input_weight{
      (Eigen::Matrix<double, 4, 1>() << 0.5, 0.6, 0.6, 0.6).finished()};
  // 仅对预测时域最后一个误差状态 Q_H 施加的比例系数。
  double terminal_weight_scale{1.0};
  // 世界系重力常数 g [m/s^2]。
  double gravity{9.805};
};

/** @brief 最近一次 OSQP 在线求解的诊断信息。 */
struct OmMpcDiagnostics
{
  // 只有 OSQP 返回 solved/solved inaccurate 且解指针有效时为 true。
  bool solved{false};
  // OSQP 原始状态字符串，或 setup 阶段的错误说明。
  std::string status;
  // OSQP ADMM 迭代次数。
  int iterations{0};
  // OSQP 报告的内部总运行时间 [ms]。
  double solve_time_ms{0.0};
  // 最终 QP 目标函数值。
  double objective{0.0};
};

/**
 * @brief 基于 SO(3) 局部误差模型的在线线性时变 MPC。
 *
 * 它和工程原 QuadControl 具有相同入口及底层输出接口，最终直接输出
 *   [总拉力 F_T, 机体系角速度 omega]。
 * 离线轨迹中的 a_T 是加速度量，发送给底层前乘无人机质量：F_T=m*a_T。
 */
class OmMpcControl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit OmMpcControl(PX4ControlNode &node, const OmMpcOptions &options = {});

  // 每个控制周期调用一次：准备 H+1 个参考节点、线性化并凝聚动力学、
  // 构造/求解 OSQP，最后写入 control_setpoint。
  px4debug_msgs::msg::Px4ctrlDebug calculateControl(
    const Ref_State_t &reference,
    const LocalPose_Data_t &pose,
    const Attitude_Data_t &attitude,
    const Sensor_Data_t &sensor,
    const double &dt,
    Control_Setpoint_t &control_setpoint,
    const Parameter_t &parameters);

  void setOptions(const OmMpcOptions &options);
  const OmMpcOptions &options() const { return options_; }
  const OmMpcDiagnostics &diagnostics() const { return diagnostics_; }

  // 设置外部给出的 H+1 点参考预瞄；设置后停用当前离线轨迹。
  // 若既没有轨迹也没有足够预瞄点，calculateControl 会从 Ref_State_t
  // 的 p/v/a/jerk/yaw_rate 在线外推一条动力学一致的短参考。
  void setReferencePreview(
    const std::vector<OmMpcReference, Eigen::aligned_allocator<OmMpcReference>> &preview);
  // 激活离线最短时间轨迹；start_time 定义轨迹 t=0 对应的 ROS 时刻。
  void setTrajectory(const OmTrajectoryResult &trajectory, const rclcpp::Time &start_time);
  // 生成器结果不再由调用者使用时可移动装载，避免复制整条离散轨迹。
  void setTrajectory(OmTrajectoryResult &&trajectory, const rclcpp::Time &start_time);
  // 清除轨迹与外部预瞄，使下一周期回到 Ref_State_t 外推模式。
  void clearTrajectory();

  // 清空诊断并把故障回退输入重置为悬停 [g,0,0,0]。
  void resetControlParams();
  // 为兼容工程现有控制器接口保留；本控制器不使用油门辨识映射。
  void resetThrustMapping(const Parameter_t &parameters);
  // 本控制器不在线估计 thrust model，恒返回 false。
  bool estimateThrustModel(const Eigen::Vector3d &estimated_acceleration);
  void init_filters(const Parameter_t &) {}
  void reset_filters() {}

private:
  // 用于读取 ROS 时钟以及保持和现有控制框架一致的节点引用。
  PX4ControlNode &node_;
  // 经 setOptions 基本裁剪后的 MPC 参数。
  OmMpcOptions options_;
  // 最近一次控制周期的求解状态。
  OmMpcDiagnostics diagnostics_;
  // 复用的调试消息，字段在 calculateControl 末尾刷新。
  px4debug_msgs::msg::Px4ctrlDebug debug_message_;
  // 外部参考预瞄；优先级低于已激活的离线轨迹。
  std::vector<OmMpcReference, Eigen::aligned_allocator<OmMpcReference>> preview_;
  // 离线轨迹副本及其 ROS 起始时刻。
  OmTrajectoryResult trajectory_;
  rclcpp::Time trajectory_start_;
  bool trajectory_active_{false};
  // OSQP 失败时保持上一周期的可用输入；顺序为 [a_T,omega]。
  Eigen::Matrix<double, 4, 1> last_input_{Eigen::Matrix<double, 4, 1>::Zero()};
};

#endif  // PX4CTRL_OMMPC_H_
