#ifndef PX4CTRL_SOLVER_NMPC_H_
#define PX4CTRL_SOLVER_NMPC_H_

#include <px4ctrl/controller.h>
#include <px4ctrl/omtraj.h>

#include <Eigen/Core>
#include <rclcpp/time.hpp>

#include <memory>
#include <string>

class PX4ControlNode;

enum class NmpcBackend
{
  // 共享 Eigen 动力学滚动计算，Ipopt 用有限差分梯度求解单重射击 NLP。
  IpoptEigen,
  // acados 代码生成的多重射击 SQP；动力学和状态均进入优化变量。
  Acados,
  // 共享 Eigen 动力学滚动计算，NLopt/L-BFGS 求解单重射击 NLP。
  NloptEigen,
  // CasADi 符号建模和自动微分，Ipopt 求解单重射击 NLP。
  IpoptCasadi,
};

// NMPC 的离散优化问题约定（四个后端必须保持一致）：
//
//   状态：x_k = [p_k^T, v_k^T, q_k^T]^T，q=[q_w,q_x,q_y,q_z]，共 10 维；
//   输入：u_k = [a_T,k, omega_x,k, omega_y,k, omega_z,k]^T，共 4 维；
//   时域：k=0,...,N-1，预测长度 T=N*dt；
//   约束：a_T,min <= a_T,k <= a_T,max，|omega_i,k| <= omega_i,max。
//
// a_T 是“质量归一化总推力”，单位 m/s^2，不是牛顿；只有向飞控发布时才乘
// 飞机质量得到 F_T=m*a_T。这样动力学模型不依赖具体机重。
struct SolverNmpcOptions
{
  // N：预测步数。acados 后端的 N 由生成代码固定，修改后需重新运行生成脚本。
  int horizon{12};
  // dt：每个预测步的离散时间 [s]。
  double prediction_dt{0.04};
  // g：重力加速度绝对值 [m/s^2]，世界系重力向量为 [0,0,-g]^T。
  double gravity{9.805};
  // 输入盒约束中的质量归一化推力上下界 [m/s^2]。
  double thrust_acceleration_min{0.5};
  double thrust_acceleration_max{4.0 * 9.805};
  // 三轴机体系角速度绝对值上界 [rad/s]。
  Eigen::Vector3d body_rate_max{Eigen::Vector3d::Constant(14.0)};
  // 后端允许的最大 NLP 迭代/函数评估次数。
  int maximum_iterations{60};
};

struct SolverNmpcDiagnostics
{
  bool solved{false};
  std::string backend;
  std::string status;
  int iterations{0};
  double solve_time_ms{0.0};
  double objective{0.0};
};

// 四种非线性求解后端的公共控制器外壳。
//
// 公共层负责：
//   1. 把 OmTrajectoryResult 或 Ref_State_t 变成长度 N+1 的参考序列；
//   2. 读取当前 p/v/q，调用选定求解器，并只执行最优序列的第一项 u_0^*；
//   3. 做输入限幅、求解失败回退、F_T=m*a_T 单位换算和调试消息填充。
//
// 这就是滚动时域控制（receding-horizon control）：下一控制周期会用新测量状态
// 再求一次，而不是一次性把整条最优输入序列发送给飞控。
class SolverNmpcControl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~SolverNmpcControl();

  SolverNmpcControl(const SolverNmpcControl &) = delete;
  SolverNmpcControl &operator=(const SolverNmpcControl &) = delete;

  px4debug_msgs::msg::Px4ctrlDebug calculateControl(
    const Ref_State_t &reference,
    const LocalPose_Data_t &pose,
    const Attitude_Data_t &attitude,
    const Sensor_Data_t &sensor,
    const double &dt,
    Control_Setpoint_t &control_setpoint,
    const Parameter_t &parameters);

  void setOptions(const SolverNmpcOptions &options);
  const SolverNmpcOptions &options() const;
  const SolverNmpcDiagnostics &diagnostics() const;

  // 保存整条连续时间轨迹和统一的起始时刻。每次 calculateControl() 只在
  // t_now+k*dt (k=0,...,N) 采样当前预测窗，不需要外部逐点喂给 MPC。
  void setTrajectory(const OmTrajectoryResult &trajectory, const rclcpp::Time &start_time);
  void setTrajectory(OmTrajectoryResult &&trajectory, const rclcpp::Time &start_time);
  void clearTrajectory();
  void resetControlParams();
  void resetThrustMapping(const Parameter_t &) {}
  bool estimateThrustModel(const Eigen::Vector3d &) {return false;}
  void init_filters(const Parameter_t &) {}
  void reset_filters() {}

protected:
  SolverNmpcControl(PX4ControlNode &node, NmpcBackend backend);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class IpoptEigenNmpcControl final : public SolverNmpcControl
{
public:
  explicit IpoptEigenNmpcControl(PX4ControlNode &node);
};

class AcadosNmpcControl final : public SolverNmpcControl
{
public:
  explicit AcadosNmpcControl(PX4ControlNode &node);
};

class NloptEigenNmpcControl final : public SolverNmpcControl
{
public:
  explicit NloptEigenNmpcControl(PX4ControlNode &node);
};

class IpoptCasadiNmpcControl final : public SolverNmpcControl
{
public:
  explicit IpoptCasadiNmpcControl(PX4ControlNode &node);
};

#endif  // PX4CTRL_SOLVER_NMPC_H_
