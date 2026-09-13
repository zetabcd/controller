#include "solver_nmpc_backend.h"

#include <px4ctrl/px4ctrl_node.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

/*
  - 状态方程、四元数运动学及离散化公式：[DYN-*]
  - 阶段代价、终端代价和姿态误差：[COST-*]
  - 输入约束、滚动时域和 warm start：[CONSTRAINT-*]、[RHC-*]、[WARM-*]
  - Ipopt/NLopt 有限差分梯度公式：[FD-1]
  - CasADi 符号 NLP 与自动微分
  - acados 多重射击、ERK、SQP、Gauss-Newton 和 HPIPM
  - 明确标注了 acados 当前没有 Δu 代价、姿态使用四元数分量残差的区别
  - acados 生成脚本也加入了对应公式说明
*/

namespace
{
using ReferenceVector = std::vector<SolverNmpcReference,
  Eigen::aligned_allocator<SolverNmpcReference>>;

double quaternionCost(
  const Eigen::Quaterniond &current, const Eigen::Quaterniond &desired)
{
  // [COST-ATT] 单位四元数姿态误差：
  //   l_q(q,q_d) = 1 - <q,q_d>^2 = sin^2(theta/2).
  // 平方点积令 l_q(q,q_d)=l_q(q,-q_d)，消除 q 与 -q 表示同一姿态的二义性；
  // theta 是两个姿态间的最短旋转角。clamp 只抑制浮点舍入越界。
  const double dot = std::clamp(current.dot(desired), -1.0, 1.0);
  return 1.0 - dot * dot;
}

double yawOf(const Eigen::Quaterniond &q)
{
  const Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

Eigen::Matrix3d attitudeFromForce(const Eigen::Vector3d &force, double yaw)
{
  // [REF-ATT-1] 平动所需合力方向决定机体 z 轴：b3_d=f_d/||f_d||。
  const Eigen::Vector3d b3 = force.norm() > 1.0e-8 ?
    force.normalized() : Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  // [REF-ATT-2] 给定期望偏航的水平朝向 b1_c=[cos(psi),sin(psi),0]^T，
  // 再正交构造 b2_d=(b3_d x b1_c)/||.||、b1_d=b2_d x b3_d。
  Eigen::Vector3d b2 = b3.cross(heading);
  if (b2.norm() < 1.0e-8) {
    b2 = b3.cross(Eigen::Vector3d::UnitY());
  }
  b2.normalize();
  Eigen::Matrix3d rotation;
  rotation.col(2) = b3;
  rotation.col(1) = b2;
  rotation.col(0) = b2.cross(b3).normalized();
  return rotation;
}

Eigen::Vector3d logSo3(const Eigen::Matrix3d &rotation)
{
  // [REF-RATE] SO(3) 对数映射：Log(R)=theta*[axis]_x，此函数返回
  // vee(Log(R))=theta*axis；相邻参考姿态的平均体角速度约为 Log(R_k^T R_{k+1})/dt。
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite()) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.angle() * angle_axis.axis();
}

const char *backendName(NmpcBackend backend)
{
  switch (backend) {
    case NmpcBackend::IpoptEigen: return "ipopt+eigen";
    case NmpcBackend::Acados: return "acados";
    case NmpcBackend::NloptEigen: return "nlopt+eigen";
    case NmpcBackend::IpoptCasadi: return "ipopt+casadi";
  }
  return "unknown";
}

std::unique_ptr<SolverNmpcBackendBase> createBackend(
  NmpcBackend backend, const SolverNmpcOptions &options)
{
  switch (backend) {
    case NmpcBackend::IpoptEigen: return createIpoptEigenBackend(options);
    case NmpcBackend::Acados: return createAcadosBackend(options);
    case NmpcBackend::NloptEigen: return createNloptEigenBackend(options);
    case NmpcBackend::IpoptCasadi: return createIpoptCasadiBackend(options);
  }
  return nullptr;
}
}  // namespace

SolverNmpcState propagateSolverNmpcState(
  const SolverNmpcState &state, const SolverNmpcCommand &control,
  double dt, double gravity)
{
  SolverNmpcState next;
  // 连续平移动力学 [DYN-1]：
  //   p_dot = v,
  //   v_dot = R(q)e_3*a_T - g*e_3,  e_3=[0,0,1]^T。
  // “state.attitude * vector” 即 Eigen 对向量执行 R(q) 旋转。
  const Eigen::Vector3d acceleration =
    state.attitude * Eigen::Vector3d(0.0, 0.0, control.thrust_acceleration) +
    Eigen::Vector3d(0.0, 0.0, -gravity);
  // 常加速度离散化 [DYN-2]：
  //   p_{k+1}=p_k+dt*v_k+0.5*dt^2*a_k，
  //   v_{k+1}=v_k+dt*a_k。
  next.position = state.position + dt * state.velocity + 0.5 * dt * dt * acceleration;
  next.velocity = state.velocity + dt * acceleration;
  // 连续四元数运动学 [DYN-3]（右乘约定，omega 为机体系角速度）：
  //   q_dot = 1/2 * q tensor [0,omega]^T。
  // 下方 q_dot 先计算未乘 1/2 的 Hamilton 积，离散时统一乘 0.5*dt。
  const Eigen::Quaterniond omega(
    0.0, control.body_rate.x(), control.body_rate.y(), control.body_rate.z());
  const Eigen::Quaterniond q_dot = state.attitude * omega;
  // 显式 Euler 离散及单位化 [DYN-4]：
  //   q_{k+1}=normalize(q_k+dt*q_dot_k)。
  // normalize 抑制数值积分造成的单位范数漂移。
  next.attitude = Eigen::Quaterniond(
    state.attitude.w() + 0.5 * dt * q_dot.w(),
    state.attitude.x() + 0.5 * dt * q_dot.x(),
    state.attitude.y() + 0.5 * dt * q_dot.y(),
    state.attitude.z() + 0.5 * dt * q_dot.z()).normalized();
  return next;
}

double solverNmpcObjective(
  const SolverNmpcState &initial_state, const ReferenceVector &references,
  const double *controls, const SolverNmpcOptions &options,
  const SolverNmpcCommand &previous_command)
{
  if (controls == nullptr ||
    references.size() != static_cast<std::size_t>(options.horizon + 1))
  {
    return 1.0e30;
  }
  // [COST-1] 单重射击决策量：
  //   U=[u_0^T,...,u_{N-1}^T]^T in R^(4N)。
  // 状态不作为独立决策量，而是从测量 x_0 反复调用 [DYN-1..4] 滚动得到。
  SolverNmpcState predicted = initial_state;
  SolverNmpcCommand previous = previous_command;
  double cost = 0.0;
  for (int k = 0; k < options.horizon; ++k) {
    const int offset = 4 * k;
    SolverNmpcCommand control;
    control.thrust_acceleration = controls[offset];
    control.body_rate = Eigen::Vector3d(
      controls[offset + 1], controls[offset + 2], controls[offset + 3]);
    predicted = propagateSolverNmpcState(
      predicted, control, options.prediction_dt, options.gravity);
    const auto &state_ref = references[static_cast<std::size_t>(k + 1)].state;
    const auto &input_ref = references[static_cast<std::size_t>(k)];
    // 阶段状态代价 [COST-2]：
    //   l_x,k = 18||p_{k+1}-p^r_{k+1}||^2
    //           +3||v_{k+1}-v^r_{k+1}||^2
    //           +5(1-<q_{k+1},q^r_{k+1}>^2)。
    // 这里状态先传播再与 k+1 参考比较，输入 u_k 与第 k 段参考比较。
    cost += 18.0 * (predicted.position - state_ref.position).squaredNorm();
    cost += 3.0 * (predicted.velocity - state_ref.velocity).squaredNorm();
    cost += 5.0 * quaternionCost(predicted.attitude, state_ref.attitude);
    const double thrust_error =
      control.thrust_acceleration - input_ref.thrust_acceleration;
    // 阶段输入跟踪代价 [COST-3]：
    //   l_u,k = 0.10(a_T,k-a^r_T,k)^2 + 0.12||omega_k-omega^r_k||^2。
    cost += 0.10 * thrust_error * thrust_error;
    cost += 0.12 * (control.body_rate - input_ref.body_rate).squaredNorm();
    const double thrust_delta =
      control.thrust_acceleration - previous.thrust_acceleration;
    // 输入变化率正则项 [COST-4]：
    //   l_Delta u,k = 0.02(a_T,k-a_T,k-1)^2
    //                 +0.12||omega_k-omega_k-1||^2。
    // k=0 时 u_{-1} 使用上个控制周期实际求得并发送的命令，减少周期间跳变。
    cost += 0.02 * thrust_delta * thrust_delta;
    cost += 0.12 * (control.body_rate - previous.body_rate).squaredNorm();
    previous = control;
  }
  const auto &terminal = references.back().state;
  // 终端代价 [COST-5]：
  //   l_N = 35||p_N-p^r_N||^2 + 6||v_N-v^r_N||^2
  //         +10(1-<q_N,q^r_N>^2)。
  cost += 35.0 * (predicted.position - terminal.position).squaredNorm();
  cost += 6.0 * (predicted.velocity - terminal.velocity).squaredNorm();
  cost += 10.0 * quaternionCost(predicted.attitude, terminal.attitude);
  return cost;
}

class SolverNmpcControl::Impl
{
public:
  Impl(PX4ControlNode &node_in, NmpcBackend backend_in)
  : node(node_in), backend_kind(backend_in)
  {
    diagnostics.backend = backendName(backend_kind);
    last_command.thrust_acceleration = options.gravity;
  }

  void rebuildBackend()
  {
    backend = createBackend(backend_kind, options);
    diagnostics = {};
    diagnostics.backend = backendName(backend_kind);
  }

  ReferenceVector makeReferences(const Ref_State_t &reference)
  {
    ReferenceVector references;
    references.reserve(static_cast<std::size_t>(options.horizon + 1));
    if (trajectory_active) {
      // [REF-1] 已加载连续轨迹时，只采样当前预测窗：
      //   t_k=(t_now-t_start)+k*dt, k=0,...,N。
      // trajectory 保存完整轨迹；MPC 每周期重采样 N+1 点，所以无需搬运全部离散点。
      const double elapsed = (node.get_clock()->now() - trajectory_start).seconds();
      for (int k = 0; k <= options.horizon; ++k) {
        const OmTrajectoryState sample = OmTrajectoryOptimizer::sample(
          trajectory, elapsed + k * options.prediction_dt);
        SolverNmpcReference output;
        output.state.position = sample.position;
        output.state.velocity = sample.velocity;
        output.state.attitude = sample.attitude.normalized();
        output.thrust_acceleration = sample.thrust_acceleration;
        output.body_rate = sample.body_rate;
        references.push_back(output);
      }
    } else {
      // [REF-2] 无完整轨迹时，对上游单点参考作局部 Taylor 外推：
      //   p^r(t)=p_0+v_0*t+1/2*a_0*t^2，v^r(t)=v_0+a_0*t，
      //   a^r(t)=a_0+j_0*t，psi^r(t)=psi_0+psi_dot*t。
      const double yaw0 = yawOf(reference.q);
      std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> rotations;
      rotations.reserve(static_cast<std::size_t>(options.horizon + 1));
      for (int k = 0; k <= options.horizon; ++k) {
        const double time = k * options.prediction_dt;
        SolverNmpcReference output;
        output.state.position = reference.p + time * reference.v +
          0.5 * time * time * reference.a;
        output.state.velocity = reference.v + time * reference.a;
        const Eigen::Vector3d acceleration = reference.a + time * reference.j;
        // [REF-3] 由平动逆动力学得到质量归一化合力：
        //   f_d=a^r+g*e3，a_T^r=||f_d||，R_d e3=f_d/||f_d||。
        const Eigen::Vector3d force = acceleration +
          options.gravity * Eigen::Vector3d::UnitZ();
        const Eigen::Matrix3d rotation = attitudeFromForce(
          force, yaw0 + time * reference.yaw_rate);
        output.state.attitude = Eigen::Quaterniond(rotation);
        output.thrust_acceleration = std::clamp(
          force.norm(), options.thrust_acceleration_min,
          options.thrust_acceleration_max);
        rotations.push_back(rotation);
        references.push_back(output);
      }
      // [REF-4] 用相邻姿态的 SO(3) 对数映射计算体角速度参考，见 [REF-RATE]。
      for (int k = 0; k < options.horizon; ++k) {
        references[static_cast<std::size_t>(k)].body_rate = logSo3(
          rotations[static_cast<std::size_t>(k)].transpose() *
          rotations[static_cast<std::size_t>(k + 1)]) / options.prediction_dt;
      }
      references.back().body_rate = options.horizon > 0 ?
        references[references.size() - 2].body_rate : Eigen::Vector3d::Zero();
    }
    return references;
  }

  PX4ControlNode &node;
  NmpcBackend backend_kind;
  SolverNmpcOptions options;
  SolverNmpcDiagnostics diagnostics;
  std::unique_ptr<SolverNmpcBackendBase> backend;
  OmTrajectoryResult trajectory;
  rclcpp::Time trajectory_start;
  bool trajectory_active{false};
  SolverNmpcCommand last_command;
  px4debug_msgs::msg::Px4ctrlDebug debug_message;
};

SolverNmpcControl::SolverNmpcControl(PX4ControlNode &node, NmpcBackend backend)
: impl_(std::make_unique<Impl>(node, backend))
{
}

SolverNmpcControl::~SolverNmpcControl() = default;

IpoptEigenNmpcControl::IpoptEigenNmpcControl(PX4ControlNode &node)
: SolverNmpcControl(node, NmpcBackend::IpoptEigen) {}
AcadosNmpcControl::AcadosNmpcControl(PX4ControlNode &node)
: SolverNmpcControl(node, NmpcBackend::Acados) {}
NloptEigenNmpcControl::NloptEigenNmpcControl(PX4ControlNode &node)
: SolverNmpcControl(node, NmpcBackend::NloptEigen) {}
IpoptCasadiNmpcControl::IpoptCasadiNmpcControl(PX4ControlNode &node)
: SolverNmpcControl(node, NmpcBackend::IpoptCasadi) {}

void SolverNmpcControl::setOptions(const SolverNmpcOptions &options)
{
  impl_->options = options;
  impl_->options.horizon = std::max(1, options.horizon);
  impl_->options.prediction_dt = std::max(1.0e-4, options.prediction_dt);
  impl_->options.maximum_iterations = std::max(1, options.maximum_iterations);
  impl_->last_command.thrust_acceleration = impl_->options.gravity;
  impl_->last_command.body_rate.setZero();
  impl_->rebuildBackend();
}

const SolverNmpcOptions &SolverNmpcControl::options() const {return impl_->options;}
const SolverNmpcDiagnostics &SolverNmpcControl::diagnostics() const
{
  return impl_->diagnostics;
}

void SolverNmpcControl::setTrajectory(
  const OmTrajectoryResult &trajectory, const rclcpp::Time &start_time)
{
  // 复制版本适合调用方仍需保留 trajectory 的情况；轨迹起点采用 ROS 时钟，
  // 必须与 calculateControl() 中 node.get_clock()->now() 属于同一时钟域。
  impl_->trajectory = trajectory;
  impl_->trajectory_start = start_time;
  impl_->trajectory_active = trajectory.success && !trajectory.states.empty();
}

void SolverNmpcControl::setTrajectory(
  OmTrajectoryResult &&trajectory, const rclcpp::Time &start_time)
{
  // 移动版本避免复制可能很长的轨迹状态数组；之后的采样逻辑完全相同。
  impl_->trajectory = std::move(trajectory);
  impl_->trajectory_start = start_time;
  impl_->trajectory_active =
    impl_->trajectory.success && !impl_->trajectory.states.empty();
}

void SolverNmpcControl::clearTrajectory()
{
  impl_->trajectory_active = false;
  impl_->trajectory.states.clear();
}

void SolverNmpcControl::resetControlParams()
{
  impl_->diagnostics = {};
  impl_->diagnostics.backend = backendName(impl_->backend_kind);
  impl_->last_command.thrust_acceleration = impl_->options.gravity;
  impl_->last_command.body_rate.setZero();
  if (impl_->backend) {
    impl_->backend->reset();
  }
}

px4debug_msgs::msg::Px4ctrlDebug SolverNmpcControl::calculateControl(
  const Ref_State_t &reference, const LocalPose_Data_t &pose,
  const Attitude_Data_t &attitude, const Sensor_Data_t &, const double &,
  Control_Setpoint_t &control_setpoint, const Parameter_t &parameters)
{
  constexpr state kManualState = 1;
  if (reference.fsm_state == kManualState) {
    // 手动姿态模式不进入 NMPC。小角度四元数关系为
    // q_error.vec() ~= 0.5*theta，故 omega_cmd=8*q_vec 相当于姿态比例增益约 4。
    Eigen::Quaterniond error = attitude.q.conjugate() * reference.q;
    if (error.w() < 0.0) {error.coeffs() *= -1.0;}
    control_setpoint.bodyrates = 8.0 * error.vec();
    control_setpoint.bodyrates.z() += reference.yaw_rate;
    const double throttle = std::clamp(reference.throttle, 0.0, 1.0);
    control_setpoint.thrust = 4.0 *
      (parameters.motor.u_min + throttle *
      (parameters.motor.u_max - parameters.motor.u_min));
    control_setpoint.q = reference.q;
    control_setpoint.rate_dot_ref.setZero();
    impl_->diagnostics.solved = true;
    impl_->diagnostics.status = "manual passthrough";
    return impl_->debug_message;
  }

  ReferenceVector references = impl_->makeReferences(reference);
  // [QUAT-SIGN] 四元数参考符号与当前估计对齐，并沿预测窗连续，避免 acados
  // 的分量二次代价把物理上相同的 q/-q 误认为大误差；Eigen/CasADi 的
  // [COST-ATT] 自身已符号不变，这里仍统一参考数据。完整 360 度路径保持连续绕行。
  if (!references.empty() &&
    attitude.q.coeffs().dot(references.front().state.attitude.coeffs()) < 0.0)
  {
    references.front().state.attitude.coeffs() *= -1.0;
  }
  for (std::size_t k = 1; k < references.size(); ++k) {
    if (references[k - 1].state.attitude.coeffs().dot(
        references[k].state.attitude.coeffs()) < 0.0)
    {
      references[k].state.attitude.coeffs() *= -1.0;
    }
  }

  SolverNmpcState current;
  current.position = pose.p;
  current.velocity = pose.v;
  current.attitude = attitude.q.normalized();
  const auto wall_start = std::chrono::steady_clock::now();
  SolverNmpcSolveResult result;
  if (impl_->backend) {
    result = impl_->backend->solve(current, references);
  } else {
    result.status = "backend construction failed";
  }
  const double wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - wall_start).count();
  impl_->diagnostics.solved = result.success;
  impl_->diagnostics.status = result.status;
  impl_->diagnostics.iterations = result.iterations;
  impl_->diagnostics.solve_time_ms =
    result.solve_time_ms > 0.0 ? result.solve_time_ms : wall_ms;
  impl_->diagnostics.objective = result.objective;
  // [RHC-1] 求解成功只保存并执行 U^* 的第一项 u_0^*；失败时保留上周期命令。
  // 后端内部会把 u_1^*,...,u_{N-1}^* 左移，作为下周期 warm start。
  if (result.success && std::isfinite(result.command.thrust_acceleration) &&
    result.command.body_rate.allFinite())
  {
    impl_->last_command = result.command;
  }
  // [CONSTRAINT-1] 发布前再次执行与 NLP 相同的盒约束，防止求解器异常值越界。
  impl_->last_command.thrust_acceleration = std::clamp(
    impl_->last_command.thrust_acceleration,
    impl_->options.thrust_acceleration_min,
    impl_->options.thrust_acceleration_max);
  impl_->last_command.body_rate = impl_->last_command.body_rate.cwiseMax(
    -impl_->options.body_rate_max).cwiseMin(impl_->options.body_rate_max);

  // [OUTPUT-1] 模型输入是 a_T=F_T/m，飞控接口需要总推力 [N]：F_T=m*a_T。
  control_setpoint.thrust = parameters.uav.mass * impl_->last_command.thrust_acceleration;
  control_setpoint.bodyrates = impl_->last_command.body_rate;
  control_setpoint.q = references.front().state.attitude;
  control_setpoint.rate_dot_ref.setZero();

  auto &debug = impl_->debug_message;
  debug.ref_p_x = references.front().state.position.x();
  debug.ref_p_y = -references.front().state.position.y();
  debug.ref_p_z = -references.front().state.position.z();
  debug.ref_v_x = references.front().state.velocity.x();
  debug.ref_v_y = -references.front().state.velocity.y();
  debug.ref_v_z = -references.front().state.velocity.z();
  debug.des_q_w = references.front().state.attitude.w();
  debug.des_q_x = references.front().state.attitude.x();
  debug.des_q_y = -references.front().state.attitude.y();
  debug.des_q_z = -references.front().state.attitude.z();
  debug.des_rate_x = control_setpoint.bodyrates.x();
  debug.des_rate_y = -control_setpoint.bodyrates.y();
  debug.des_rate_z = -control_setpoint.bodyrates.z();
  debug.des_thrust = control_setpoint.thrust;
  debug.timestamp = impl_->node.get_clock()->now().nanoseconds() / 1000;
  return debug;
}
