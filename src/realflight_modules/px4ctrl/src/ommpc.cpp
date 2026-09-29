#include <px4ctrl/ommpc.h>

#include <px4ctrl/px4ctrl_node.h>

#include <Eigen/SparseCore>
#include <osqp/osqp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace
{
/*
 * OMMPC 数学公式—代码位置对照
 *
 * (M1)  x=(p,v,R), u=(a_T,omega)                    -> OmMpcReference
 * (M2)  e=[p-pd,v-vd,Log(Rd^T R)]                   -> stateError()
 * (M3)  p+=h v, v+=h(-g e3+a_T R e3),
 *       R+=R Exp(h omega^)                           -> linearizeStep() 内 propagate_error
 * (M4)  e_{k+1}=A_k e_k+B_k delta_u_k+c_k           -> linearizeStep()
 * (M5)  E=S_x e_0+S_u Delta_U+c                      -> calculateControl() 凝聚循环
 * (M6)  J=E^T Qbar E+Delta_U^T Rbar Delta_U          -> h_dense/gradient
 * (M7)  u_min-u_d<=delta_u<=u_max-u_d                -> lower/upper
 * (M8)  Delta_U*=argmin 1/2 U^T H U+q^T U            -> solveOsqp()
 * (M9)  u_cmd=u_0^d+delta_u_0^*                      -> command
 * (M10) F_T=m a_T                                    -> control_setpoint.thrust
 *
 * 注意：这里使用“误差动力学凝聚”而非把所有预测状态都作为 QP 变量，
 * 因此 OSQP 的决策维数是 4H，而不是 9H+4H。
 */

// 9 维局部误差 [delta_p,delta_v,delta_theta]。
using Vector9 = Eigen::Matrix<double, 9, 1>;
// 单步误差状态矩阵 A_k。
using Matrix9 = Eigen::Matrix<double, 9, 9>;
// 单步输入修正矩阵 B_k，输入为 [delta_a_T,delta_omega]。
using Matrix94 = Eigen::Matrix<double, 9, 4>;

// SO(3) 帽映射：hat(v)w=v x w。
Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Matrix3d expSo3(const Eigen::Vector3d &v)
{
  // Rodrigues/AngleAxis 指数映射。小角度时用 I+hat(v)，避免 v/||v|| 数值放大。
  const double angle = v.norm();
  if (angle < 1.0e-10) {
    return Eigen::Matrix3d::Identity() + hat(v);
  }
  return Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}

Eigen::Vector3d logSo3(const Eigen::Matrix3d &rotation)
{
  // SO(3) 对数映射返回最短旋转向量；它是姿态误差 delta_theta 的局部坐标。
  Eigen::AngleAxisd aa(rotation);
  if (!std::isfinite(aa.angle()) || aa.angle() < 1.0e-10) {
    return Eigen::Vector3d::Zero();
  }
  double angle = aa.angle();
  Eigen::Vector3d axis = aa.axis();
  if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  return angle * axis;
}

struct CscStorage
{
  // OSQP C API 的矩阵由动态内存持有。该 RAII 包装负责所有权转移和释放，
  // 并禁止复制，防止同一 OSQPCscMatrix 被释放两次。
  CscStorage() = default;
  CscStorage(const CscStorage &) = delete;
  CscStorage &operator=(const CscStorage &) = delete;
  CscStorage(CscStorage &&other) noexcept
  : values(std::move(other.values)), rows(std::move(other.rows)),
    columns(std::move(other.columns)), matrix(other.matrix)
  {
    other.matrix = nullptr;
  }
  std::vector<OSQPFloat> values;
  std::vector<OSQPInt> rows;
  std::vector<OSQPInt> columns;
  OSQPCscMatrix *matrix{nullptr};

  ~CscStorage()
  {
    if (matrix != nullptr) {
      OSQPCscMatrix_free(matrix);
    }
  }
};

CscStorage toCsc(const Eigen::SparseMatrix<double> &input, bool upper_triangle)
{
  // Eigen 稀疏矩阵转换为 OSQP 所需的 CSC 三数组：
  // values=非零值，rows=对应行号，columns=每列起始偏移。
  // OSQP 对 Hessian P 只读取上三角，因此 upper_triangle=true 时丢弃下三角。
  Eigen::SparseMatrix<double> compressed = input;
  compressed.makeCompressed();
  CscStorage result;
  result.columns.resize(static_cast<std::size_t>(compressed.cols() + 1), 0);
  for (int col = 0; col < compressed.outerSize(); ++col) {
    result.columns[static_cast<std::size_t>(col)] =
      static_cast<OSQPInt>(result.values.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(compressed, col); it; ++it) {
      if (upper_triangle && it.row() > it.col()) {
        continue;
      }
      if (std::abs(it.value()) < 1.0e-12) {
        continue;
      }
      result.rows.push_back(static_cast<OSQPInt>(it.row()));
      result.values.push_back(static_cast<OSQPFloat>(it.value()));
    }
  }
  result.columns.back() = static_cast<OSQPInt>(result.values.size());
  result.matrix = OSQPCscMatrix_new(
    static_cast<OSQPInt>(compressed.rows()), static_cast<OSQPInt>(compressed.cols()),
    static_cast<OSQPInt>(result.values.size()), result.values.data(), result.rows.data(),
    result.columns.data());
  return result;
}

bool solveOsqp(
  const Eigen::SparseMatrix<double> &hessian, const Eigen::VectorXd &gradient,
  const Eigen::SparseMatrix<double> &constraint, const Eigen::VectorXd &lower,
  const Eigen::VectorXd &upper, Eigen::VectorXd *solution, OmMpcDiagnostics *diagnostics)
{
  // 标准形式：min_z 1/2 z^T H z+g^T z,  lower<=A z<=upper。
  // 本函数只负责数据格式和求解状态，不解释 z 的物理含义。
  CscStorage p = toCsc(hessian, true);
  CscStorage a = toCsc(constraint, false);
  std::vector<OSQPFloat> q(gradient.data(), gradient.data() + gradient.size());
  std::vector<OSQPFloat> l(lower.data(), lower.data() + lower.size());
  std::vector<OSQPFloat> u(upper.data(), upper.data() + upper.size());
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  // 在线控制不输出 OSQP 内部日志。当前 QP 通常在输入盒内部取得最优点，
  // 没有活跃集可供 polishing 精修；关闭它也避免 OSQP 每周期打印
  // "Polishing not needed - no active set detected"。
  settings.verbose = 0;
  // 该实现每周期重新 osqp_setup，且未显式注入上一周期 primal/dual 解；因此
  // warm_starting 仅允许 OSQP 内部接口，并不等于已经实现跨周期热启动。
  settings.warm_starting = 1;
  settings.polishing = 0;
  settings.max_iter = 2500;
  settings.eps_abs = 1.0e-5;
  settings.eps_rel = 1.0e-5;
  OSQPSolver *solver = nullptr;
  const OSQPInt setup_status = osqp_setup(
    &solver, p.matrix, q.data(), a.matrix, l.data(), u.data(),
    static_cast<OSQPInt>(constraint.rows()), static_cast<OSQPInt>(hessian.rows()), &settings);
  if (setup_status != 0 || solver == nullptr) {
    diagnostics->status = "OSQP setup failed: " + std::to_string(setup_status);
    return false;
  }
  const OSQPInt solve_status = osqp_solve(solver);
  // 保存本周期诊断量，便于上层判断实时性和数值质量。
  diagnostics->iterations = static_cast<int>(solver->info->iter);
  diagnostics->solve_time_ms = 1000.0 * solver->info->run_time;
  diagnostics->objective = solver->info->obj_val;
  diagnostics->status = solver->info->status;
  const bool solved = solve_status == 0 &&
    (solver->info->status_val == OSQP_SOLVED ||
    solver->info->status_val == OSQP_SOLVED_INACCURATE) && solver->solution != nullptr;
  if (solved) {
    solution->resize(hessian.rows());
    for (int i = 0; i < solution->size(); ++i) {
      (*solution)(i) = solver->solution->x[i];
    }
  }
  osqp_cleanup(solver);
  return solved;
}

Vector9 stateError(
  const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Matrix3d &r,
  const OmMpcReference &reference)
{
  // (M2) 位置和速度直接作差；姿态不能直接减四元数，而使用右不变局部误差
  // delta_theta=Log((R_d)^T R)。当 R=R_d 时三部分均为零。
  Vector9 error;
  error.segment<3>(0) = p - reference.position;
  error.segment<3>(3) = v - reference.velocity;
  error.segment<3>(6) = logSo3(reference.attitude.normalized().toRotationMatrix().transpose() * r);
  return error;
}

void linearizeStep(
  const OmMpcReference &current, const OmMpcReference &next, double dt, double gravity,
  Matrix9 *a, Matrix94 *b, Vector9 *defect)
{
  // 在名义参考 (x_k^d,u_k^d,x_{k+1}^d) 周围构造
  // e_{k+1}=A_k e_k+B_k delta_u_k+c_k。
  // c_k 是参考自身对所用离散动力学的不一致量；保留它可避免假设输入轨迹
  // 与 MPC 的 Euler 步长严格一致。
  const Eigen::Matrix3d rd = current.attitude.normalized().toRotationMatrix();
  const Eigen::Matrix3d rd_next = next.attitude.normalized().toRotationMatrix();
  const Eigen::Vector3d g(0.0, 0.0, -gravity);
  const auto propagate_error = [&](const Vector9 &dx, const Eigen::Matrix<double, 4, 1> &du) {
      // 先通过流形回缩 R=R_d Exp(delta_theta^) 构造受扰真实状态。
      const Eigen::Vector3d p = current.position + dx.segment<3>(0);
      const Eigen::Vector3d v = current.velocity + dx.segment<3>(3);
      const Eigen::Matrix3d r = rd * expSo3(dx.segment<3>(6));
      const double thrust = current.thrust_acceleration + du(0);
      const Eigen::Vector3d omega = current.body_rate + du.tail<3>();
      // (M3) 零阶保持输入的一阶离散四旋翼动力学。
      const Eigen::Vector3d pn = p + dt * v;
      const Eigen::Vector3d vn = v + dt * (g + thrust * r.col(2));
      const Eigen::Matrix3d rn = r * expSo3(dt * omega);
      Vector9 out;
      out.segment<3>(0) = pn - next.position;
      out.segment<3>(3) = vn - next.velocity;
      out.segment<3>(6) = logSo3(rd_next.transpose() * rn);
      return out;
    };
  const Vector9 zero_x = Vector9::Zero();
  const Eigen::Matrix<double, 4, 1> zero_u = Eigen::Matrix<double, 4, 1>::Zero();
  *defect = propagate_error(zero_x, zero_u);
  // 采用前向有限差分计算 A=df/de、B=df/d(delta_u)。epsilon 是局部
  // 坐标扰动量，并不代表实际控制步长。
  constexpr double epsilon = 1.0e-6;
  for (int i = 0; i < 9; ++i) {
    Vector9 delta = Vector9::Zero();
    delta(i) = epsilon;
    a->col(i) = (propagate_error(delta, zero_u) - *defect) / epsilon;
  }
  for (int i = 0; i < 4; ++i) {
    Eigen::Matrix<double, 4, 1> delta = zero_u;
    delta(i) = epsilon;
    b->col(i) = (propagate_error(zero_x, delta) - *defect) / epsilon;
  }
}
}  // namespace

OmMpcControl::OmMpcControl(PX4ControlNode &node, const OmMpcOptions &options)
: node_(node), options_(options)
{
  // 在第一次 OSQP 失败时，安全回退为零角速度悬停比全零推力更合理。
  last_input_(0) = options_.gravity;
}

void OmMpcControl::setOptions(const OmMpcOptions &options)
{
  // 只裁剪会造成空 QP 或除零的结构参数；物理上下界由调用者配置负责。
  options_ = options;
  options_.horizon = std::max(1, options_.horizon);
  options_.prediction_dt = std::max(1.0e-4, options_.prediction_dt);
}

void OmMpcControl::resetControlParams()
{
  // 飞行模式切换或控制器复位时，不应沿用上一模式中的角速度命令。
  diagnostics_ = {};
  last_input_.setZero();
  last_input_(0) = options_.gravity;
}

void OmMpcControl::resetThrustMapping(const Parameter_t &)
{
  // 本控制器和工程现有 rate controller 共用“牛顿”单位的总拉力接口；MPC
  // 内部优化 a_T=F_T/m，输出时显式乘质量，因此不需要油门—加速度辨识曲线。
}

bool OmMpcControl::estimateThrustModel(const Eigen::Vector3d &)
{
  // 为匹配原 Controller 接口保留。false 表示没有更新任何在线推力模型。
  return false;
}

px4debug_msgs::msg::Px4ctrlDebug OmMpcControl::calculateControl(
  const px4ctrl::ReferenceWindow &window,
  const Ref_State_t &reference, const LocalPose_Data_t &pose,
  const Attitude_Data_t &attitude, const Sensor_Data_t &, const double & /*dt*/,
  Control_Setpoint_t &control_setpoint, const Parameter_t &parameters)
{
  control_setpoint.rate_dot_ref_valid = false;
  // 每周期诊断只描述本次求解，不能遗留上一周期的 solved/status。
  diagnostics_ = {};
  // attitude.q 按工程约定表示机体系到世界系旋转。
  const Eigen::Matrix3d current_r = attitude.q.normalized().toRotationMatrix();

  // Preserve the existing manual-mode contract without running a position MPC
  // on NaN reference fields produced by set_manual_ref().
  // PX4CtrlFSM procedure order is manual_on=0, manual=1, auto_hover=2, ... .
  // The FSM_STATE macro creates class-local symbols, so this independent
  // controller compares against the stable public state value here.
  constexpr state kManualState = 1;
  if (reference.fsm_state == kManualState) {
    // 手动模式不构造位置 MPC。四元数误差 q^{-1}q_d 的向量部近似半角误差，
    // 因而 8*vec(q_e) 在小角度下约等于 4*theta_e。
    Eigen::Quaterniond error = attitude.q.conjugate() * reference.q;
    // q 与 -q 表示同一姿态，选择 w>=0 的短旋转分支避免角速度跳变。
    if (error.w() < 0.0) {
      error.coeffs() *= -1.0;
    }
    control_setpoint.bodyrates = 8.0 * error.vec();
    control_setpoint.bodyrates.z() += reference.yaw_rate;
    // 保持原工程手动油门契约：归一化 throttle 映射到单电机范围，再乘 4
    // 形成四电机总拉力命令 [N]。
    const double throttle = std::clamp(reference.throttle, 0.0, 1.0);
    control_setpoint.thrust = 4.0 *
      (parameters.motor.u_min + throttle * (parameters.motor.u_max - parameters.motor.u_min));
    control_setpoint.q = reference.q;
    control_setpoint.rate_dot_ref.setZero();
    diagnostics_.solved = true;
    diagnostics_.status = "manual passthrough";
    return debug_message_;
  }

  const int horizon = std::max(1, options_.horizon);
  const double prediction_dt = options_.prediction_dt;
  px4ctrl::validateReferenceWindow(window, horizon, prediction_dt);
  const auto &refs = window.points;

  // -----------------------------------------------------------------------
  // 预测模型凝聚
  // -----------------------------------------------------------------------
  // 决策向量 Delta_U=[delta_u_0;...;delta_u_{H-1}]，维数 4H。
  const int nu = 4 * horizon;
  // 堆叠误差 E=[e_1;...;e_H] 满足 E=Sx*e0+Su*Delta_U+affine。
  Eigen::MatrixXd sx = Eigen::MatrixXd::Zero(9 * horizon, 9);
  Eigen::MatrixXd su = Eigen::MatrixXd::Zero(9 * horizon, nu);
  Eigen::VectorXd affine = Eigen::VectorXd::Zero(9 * horizon);
  Matrix9 phi = Matrix9::Identity();
  Eigen::MatrixXd gamma = Eigen::MatrixXd::Zero(9, nu);
  Vector9 offset = Vector9::Zero();
  for (int k = 0; k < horizon; ++k) {
    Matrix9 ak;
    Matrix94 bk;
    Vector9 ck;
    linearizeStep(refs[k], refs[k + 1], prediction_dt, options_.gravity, &ak, &bk, &ck);
    // 递推展开：
    // Phi_k=A_k...A_0；Gamma_k=[A_k...A_1 B_0,...,B_k]；
    // offset_k=A_k offset_{k-1}+c_k。
    phi = ak * phi;
    gamma = ak * gamma;
    gamma.block<9, 4>(0, 4 * k) += bk;
    offset = ak * offset + ck;
    sx.block<9, 9>(9 * k, 0) = phi;
    su.block(9 * k, 0, 9, nu) = gamma;
    affine.segment<9>(9 * k) = offset;
  }

  // 当前真实状态相对预测首节点的 9 维流形误差 e0。
  const Vector9 x0 = stateError(pose.p, pose.v, current_r, refs.front());
  // Delta_U=0 时整段预测误差，即参考前馈输入自身产生的自由响应。
  const Eigen::VectorXd free_response = sx * x0 + affine;
  // Qbar=blkdiag(Q,...,Q_H)，最后一块可通过 terminal_weight_scale 放大。
  Eigen::MatrixXd qbar = Eigen::MatrixXd::Zero(9 * horizon, 9 * horizon);
  for (int k = 0; k < horizon; ++k) {
    const double scale = (k == horizon - 1) ? options_.terminal_weight_scale : 1.0;
    qbar.block<9, 9>(9 * k, 9 * k) = (scale * options_.state_weight).asDiagonal();
  }
  // Rbar=blkdiag(R,...,R)，只惩罚相对参考前馈输入的修正 delta_u。
  Eigen::MatrixXd rbar = Eigen::MatrixXd::Zero(nu, nu);
  for (int k = 0; k < horizon; ++k) {
    rbar.block<4, 4>(4 * k, 4 * k) = options_.input_weight.asDiagonal();
  }
  // 将 J=(f+Su U)^T Qbar(f+Su U)+U^T Rbar U 展开成 OSQP 形式：
  // H=2(Su^T Qbar Su+Rbar), q=2 Su^T Qbar f。
  Eigen::MatrixXd h_dense = 2.0 * (su.transpose() * qbar * su + rbar);
  // 极小对角正则保证数值半正定，避免未被当前预测模型激励的方向奇异。
  h_dense.diagonal().array() += 1.0e-8;
  const Eigen::VectorXd gradient = 2.0 * su.transpose() * qbar * free_response;
  Eigen::SparseMatrix<double> hessian = h_dense.sparseView();
  Eigen::SparseMatrix<double> constraint(nu, nu);
  // 目前只有输入盒约束，所以 A=I。状态/倾角等约束尚未加入在线 MPC QP。
  constraint.setIdentity();
  Eigen::VectorXd lower(nu);
  Eigen::VectorXd upper(nu);
  for (int k = 0; k < horizon; ++k) {
    // OSQP 变量是修正量，故绝对输入界 u_min<=u_d+delta_u<=u_max
    // 要平移为 u_min-u_d<=delta_u<=u_max-u_d。
    lower(4 * k) = options_.thrust_acceleration_min - refs[k].thrust_acceleration;
    upper(4 * k) = options_.thrust_acceleration_max - refs[k].thrust_acceleration;
    lower.segment<3>(4 * k + 1) = -options_.body_rate_max - refs[k].body_rate;
    upper.segment<3>(4 * k + 1) = options_.body_rate_max - refs[k].body_rate;
  }
  Eigen::VectorXd delta_input;
  // 求解完整 4H 输入修正序列，但滚动时域控制只执行第一块（receding horizon）。
  diagnostics_.solved = solveOsqp(
    hessian, gradient, constraint, lower, upper, &delta_input, &diagnostics_);
  Eigen::Matrix<double, 4, 1> command;
  if (diagnostics_.solved && delta_input.allFinite()) {
    // (M9) 真实命令=第一个参考前馈输入+QP 最优第一步修正。
    command = Eigen::Matrix<double, 4, 1>(refs.front().thrust_acceleration,
      refs.front().body_rate.x(), refs.front().body_rate.y(), refs.front().body_rate.z()) +
      delta_input.head<4>();
    last_input_ = command;
  } else {
    // 求解失败不发送 NaN/零推力；保持上一条有效命令，并再次裁剪拉力。
    command = last_input_;
    command(0) = std::clamp(command(0), options_.thrust_acceleration_min,
      options_.thrust_acceleration_max);
  }

  // 轨迹与 MPC 内部使用质量归一化拉力 a_T，底层接口使用总拉力 F_T [N]。
  control_setpoint.thrust = parameters.uav.mass * command(0);
  // 角速度已经是机体系 [rad/s]，无需质量或惯量换算。
  control_setpoint.bodyrates = command.tail<3>();
  // q 仅作为期望姿态/日志字段；本控制器的直接执行输入仍是 thrust+bodyrates。
  control_setpoint.q = refs.front().attitude;
  control_setpoint.rate_dot_ref = diagnostics_.solved ?
    px4ctrl::angularFeedforward(refs.front(), attitude.q) : Eigen::Vector3d::Zero();

  control_setpoint.rate_dot_ref_valid = diagnostics_.solved && refs.front().angular_acceleration_valid;

  // 工程调试消息沿用历史显示坐标约定：x 不变，y/z 取反；这里的符号变换
  // 只影响日志/可视化，不反馈到 MPC 内部 ENU 动力学和控制命令。
  debug_message_.ref_p_x = refs.front().position.x();
  debug_message_.ref_p_y = -refs.front().position.y();
  debug_message_.ref_p_z = -refs.front().position.z();
  debug_message_.ref_v_x = refs.front().velocity.x();
  debug_message_.ref_v_y = -refs.front().velocity.y();
  debug_message_.ref_v_z = -refs.front().velocity.z();
  debug_message_.des_q_w = refs.front().attitude.w();
  debug_message_.des_q_x = refs.front().attitude.x();
  debug_message_.des_q_y = -refs.front().attitude.y();
  debug_message_.des_q_z = -refs.front().attitude.z();
  debug_message_.des_rate_x = command(1);
  debug_message_.des_rate_y = -command(2);
  debug_message_.des_rate_z = -command(3);
  debug_message_.des_thrust = control_setpoint.thrust;
  debug_message_.ref_rate_dot_x = control_setpoint.rate_dot_ref.x();
  debug_message_.ref_rate_dot_y = -control_setpoint.rate_dot_ref.y();
  debug_message_.ref_rate_dot_z = -control_setpoint.rate_dot_ref.z();
  // Px4ctrlDebug 时间戳单位为微秒。
  debug_message_.timestamp = node_.get_clock()->now().nanoseconds() / 1000;
  return debug_message_;
}
