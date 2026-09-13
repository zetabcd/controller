#include <px4ctrl/omtraj.h>

#include <Eigen/SparseCore>
#include <osqp/osqp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <vector>

namespace
{
/*
 * 论文公式—实现对照索引（无人机部分）
 *
 * (F1)  hat(a)b = a x b                         -> hat()
 * (F2)  R=Exp(theta), theta=Log(R)               -> expSo3(), logSo3()
 * (F3)  p_dot=v, v_dot=-g e3+a_T R e3,
 *       R_dot=R hat(omega)                       -> propagate()
 * (F4)  x_{k+1}=f_d(x_k,u_k,dt)                  -> propagate()
 * (F5)  delta x_{k+1}=A delta x_k+B delta u_k
 *       -defect+nu (dt=T/N fixed per inner solve)-> linearize()+dynamics rows
 * (F6)  min 1/2 z'Hz+q'z                         -> h_triplets + gradient
 * (F7)  lambda_{k+1,j}=lambda_{k,j}-mu_{k,j}     -> progress rows
 * (F8)  mu_{k,j}(||p_k-p_j||^2-r_j^2)<=sigma_i+s -> waypoint CSTC homotopy rows
 * (F9)  lambda_{k,j}mu_{k,j+1}<=sigma_i+s        -> strict-order CSTC homotopy rows
 * (F10) |u_k-u_{k-1}|<=dot(u)_max T/N            -> input-rate rows
 * (F11) u_min<=u_k<=u_max and trust regions      -> identity bound rows
 * (F12) T*=inf{T | fixed-T constraints feasible} -> outer time bisection
 * (F13) t_j=(T/N) sum_k k mu_{k,j}               -> waypoint_times
 * (F14) alpha in {1,1/2,...}: min nonlinear merit -> SCP backtracking
 * (F15) linear interpolation + quaternion SLERP  -> sample()
 *
 * 下方每一处组装代码都再次标注相应公式，方便逐项与文章核对。
 */
// ---------------------------------------------------------------------------
// 数学模型与局部坐标
// ---------------------------------------------------------------------------
// 非线性状态为 (p,v,R) in R^3 x R^3 x SO(3)，控制为
// u=[a_T,omega]。连续动力学为
//   p_dot = v,
//   v_dot = -g*e_3 + a_T*R*e_3,
//   R_dot = R*hat(omega).
// SCP 不直接把旋转矩阵的 9 个元素当变量，而是在当前名义姿态 R_bar 的切空间
// 使用三维扰动 theta：R = R_bar*Exp(theta)。因此每个状态节点只有 9 个增量
// [delta_p,delta_v,theta]，既避免冗余，也保持更新后的姿态属于 SO(3)。
using Vector9 = Eigen::Matrix<double, 9, 1>;
using Matrix9 = Eigen::Matrix<double, 9, 9>;
using Matrix94 = Eigen::Matrix<double, 9, 4>;

Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
  // hat(v)*w = v x w，是 so(3) 李代数的反对称矩阵表示。
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Matrix3d expSo3(const Eigen::Vector3d &v)
{
  // SO(3) 指数映射：旋转向量 -> 旋转矩阵；小角度时使用一阶展开避免除零。
  const double angle = v.norm();
  if (angle < 1.0e-10) {
    return Eigen::Matrix3d::Identity() + hat(v);
  }
  return Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}

Eigen::Vector3d logSo3(const Eigen::Matrix3d &rotation)
{
  // SO(3) 对数映射：旋转矩阵 -> 最短旋转向量，用于姿态误差和角速度初始化。
  Eigen::AngleAxisd aa(rotation);
  if (!std::isfinite(aa.angle()) || aa.angle() < 1.0e-10) {
    return Eigen::Vector3d::Zero();
  }
  return aa.angle() * aa.axis();
}

struct NominalState
{
  // SCP 当前迭代的名义轨迹。QP 求出的是相对它的增量，而不是绝对状态。
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d r{Eigen::Matrix3d::Identity()};
};

struct CscStorage
{
  CscStorage() = default;
  CscStorage(const CscStorage &) = delete;
  CscStorage &operator=(const CscStorage &) = delete;
  CscStorage(CscStorage &&other) noexcept
  : values(std::move(other.values)), rows(std::move(other.rows)),
    columns(std::move(other.columns)), matrix(other.matrix)
  {
    other.matrix = nullptr;
  }
  ~CscStorage()
  {
    if (matrix != nullptr) {
      OSQPCscMatrix_free(matrix);
    }
  }
  std::vector<OSQPFloat> values;
  std::vector<OSQPInt> rows;
  std::vector<OSQPInt> columns;
  OSQPCscMatrix *matrix{nullptr};
};

CscStorage toCsc(const Eigen::SparseMatrix<double> &input, bool upper_triangle)
{
  // OSQP 接收 CSC 稀疏格式；目标 Hessian P 只需要上传上三角部分。
  Eigen::SparseMatrix<double> compressed = input;
  compressed.makeCompressed();
  CscStorage result;
  result.columns.resize(static_cast<std::size_t>(compressed.cols() + 1), 0);
  for (int col = 0; col < compressed.outerSize(); ++col) {
    result.columns[static_cast<std::size_t>(col)] =
      static_cast<OSQPInt>(result.values.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(compressed, col); it; ++it) {
      if ((upper_triangle && it.row() > it.col()) || std::abs(it.value()) < 1.0e-12) {
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
  const Eigen::VectorXd &upper, Eigen::VectorXd *solution, std::string *status)
{
  // 将标准 QP 交给 OSQP：min 1/2 z^T H z + g^T z，l<=Az<=u。
  // CscStorage 的析构函数负责释放 OSQP 分配的矩阵，避免迭代中内存累积。
  CscStorage p = toCsc(hessian, true);
  CscStorage a = toCsc(constraint, false);
  std::vector<OSQPFloat> q(gradient.data(), gradient.data() + gradient.size());
  std::vector<OSQPFloat> l(lower.data(), lower.data() + lower.size());
  std::vector<OSQPFloat> u(upper.data(), upper.data() + upper.size());
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.warm_starting = 1;
  settings.polishing = 1;
  settings.scaled_termination = 1;
  settings.adaptive_rho_interval = 50;
  settings.max_iter = 50000;
  settings.eps_abs = 5.0e-5;
  settings.eps_rel = 5.0e-5;
  OSQPSolver *solver = nullptr;
  const OSQPInt setup = osqp_setup(
    &solver, p.matrix, q.data(), a.matrix, l.data(), u.data(),
    static_cast<OSQPInt>(constraint.rows()), static_cast<OSQPInt>(hessian.rows()), &settings);
  if (setup != 0 || solver == nullptr) {
    *status = "OSQP setup failed: " + std::to_string(setup);
    return false;
  }
  const OSQPInt code = osqp_solve(solver);
  *status = std::string(solver->info->status) +
    " (prim=" + std::to_string(solver->info->prim_res) +
    ", dual=" + std::to_string(solver->info->dual_res) + ")";
  const bool accurate_status = solver->info->status_val == OSQP_SOLVED ||
    solver->info->status_val == OSQP_SOLVED_INACCURATE;
  const bool usable_iteration_limit =
    solver->info->status_val == OSQP_MAX_ITER_REACHED &&
    solver->info->prim_res <= 1.0e-4 && solver->info->dual_res <= 1.0e-4;
  const bool solved = code == 0 && (accurate_status || usable_iteration_limit) &&
    solver->solution != nullptr;
  if (solved) {
    solution->resize(hessian.rows());
    for (int i = 0; i < solution->size(); ++i) {
      (*solution)(i) = solver->solution->x[i];
    }
  }
  osqp_cleanup(solver);
  return solved;
}

Vector9 difference(const NominalState &state, const NominalState &reference)
{
  // (F2) 局部误差：dx=[p-p_bar, v-v_bar, Log(R_bar^T R)]。
  Vector9 result;
  result.segment<3>(0) = state.p - reference.p;
  result.segment<3>(3) = state.v - reference.v;
  result.segment<3>(6) = logSo3(reference.r.transpose() * state.r);
  return result;
}

NominalState propagate(
  const NominalState &state, const Eigen::Matrix<double, 4, 1> &input,
  double dt, double gravity)
{
  // (F3,F4) 零阶保持的一阶离散模型：
  // p+=dt*v, v+=dt*(-g*e3+a_T*R*e3), R+=R*Exp(dt*omega)。
  NominalState next;
  next.p = state.p + dt * state.v;
  next.v = state.v + dt *
    (Eigen::Vector3d(0.0, 0.0, -gravity) + input(0) * state.r.col(2));
  next.r = state.r * expSo3(dt * input.tail<3>());
  return next;
}

void linearize(
  const NominalState &current, const NominalState &next,
  const Eigen::Matrix<double, 4, 1> &input, double dt, double gravity,
  Matrix9 *a, Matrix94 *b, Vector9 *defect)
{
  // (F5) 数值计算离散动力学残差对状态、控制和步长 dt 的 Jacobian：
  //   delta_x[k+1] = A_k delta_x[k] + B_k delta_u[k]
  //                    - defect_k + nu_k.
  // nu_k 是高权重惩罚的虚拟控制，用于防止早期 SCP 子问题不可行。
  const auto evaluate = [&](const Vector9 &dx, const Eigen::Matrix<double, 4, 1> &du) {
      NominalState perturbed = current;
      perturbed.p += dx.segment<3>(0);
      perturbed.v += dx.segment<3>(3);
      perturbed.r = current.r * expSo3(dx.segment<3>(6));
      return difference(propagate(perturbed, input + du, dt, gravity), next);
    };
  const Vector9 zero_x = Vector9::Zero();
  const Eigen::Matrix<double, 4, 1> zero_u = Eigen::Matrix<double, 4, 1>::Zero();
  *defect = evaluate(zero_x, zero_u);
  constexpr double eps = 1.0e-6;
  for (int i = 0; i < 9; ++i) {
    Vector9 delta = zero_x;
    delta(i) = eps;
    a->col(i) = (evaluate(delta, zero_u) - *defect) / eps;
  }
  for (int i = 0; i < 4; ++i) {
    Eigen::Matrix<double, 4, 1> delta = zero_u;
    delta(i) = eps;
    b->col(i) = (evaluate(zero_x, delta) - *defect) / eps;
  }
}

Eigen::Matrix3d forceAttitude(const Eigen::Vector3d &force, double yaw)
{
  // 根据期望合力方向构造姿态：机体 z 轴与合力同向，偏航由 yaw 指定。
  // 这里只用于生成名义初值；最终姿态由动力学和控制约束共同优化得到。
  Eigen::Vector3d zb = force.norm() > 1.0e-7 ? force.normalized() : Eigen::Vector3d::UnitZ();
  Eigen::Vector3d yc(-std::sin(yaw), std::cos(yaw), 0.0);
  Eigen::Vector3d xb = yc.cross(zb);
  if (xb.norm() < 1.0e-7) {
    xb = Eigen::Vector3d::UnitX();
  } else {
    xb.normalize();
  }
  Eigen::Matrix3d r;
  r.col(0) = xb;
  r.col(2) = zb;
  r.col(1) = zb.cross(xb).normalized();
  return r;
}

double yawOf(const Eigen::Matrix3d &r)
{
  return std::atan2(r(1, 0), r(0, 0));
}

double wrapAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
}  // namespace

OmTrajectoryOptimizer::OmTrajectoryOptimizer(const OmTrajectoryOptions &options)
: options_(options)
{
  setOptions(options);
}

void OmTrajectoryOptimizer::setOptions(const OmTrajectoryOptions &options)
{
  // 在入口统一裁剪会破坏 QP 或造成负约束界的配置，保持后续组装逻辑简单。
  options_ = options;
  options_.intervals = std::max(2, options_.intervals);
  options_.max_scp_iterations = std::max(1, options_.max_scp_iterations);
  options_.maximum_time = std::max(options_.minimum_time, options_.maximum_time);
  options_.time_search_tolerance = std::max(1.0e-4, options_.time_search_tolerance);
  options_.thrust_acceleration_rate_max =
    std::max(0.0, options_.thrust_acceleration_rate_max);
  options_.body_rate_acceleration_max =
    options_.body_rate_acceleration_max.cwiseMax(0.0);
  options_.thrust_trust_region = std::max(1.0e-6, options_.thrust_trust_region);
  options_.body_rate_trust_region = options_.body_rate_trust_region.cwiseMax(1.0e-6);
  options_.scp_backtracking_steps = std::max(1, options_.scp_backtracking_steps);
  options_.cstc_warm_start_iterations =
    std::max(0, options_.cstc_warm_start_iterations);
  options_.cstc_initial_support_radius =
    std::max(0, options_.cstc_initial_support_radius);
  options_.cstc_relaxation_initial =
    std::max(0.0, options_.cstc_relaxation_initial);
  options_.cstc_relaxation_decay =
    std::clamp(options_.cstc_relaxation_decay, 0.0, 1.0);
}

OmTrajectoryResult OmTrajectoryOptimizer::optimize(
  const OmTrajectoryBoundary &initial, const OmTrajectoryBoundary &terminal,
  const std::vector<OmTrajectoryWaypoint,
    Eigen::aligned_allocator<OmTrajectoryWaypoint>> &waypoints)
{
  const auto optimization_wall_start = std::chrono::steady_clock::now();
  const auto elapsed_solve_time = [&]() {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - optimization_wall_start).count();
    };
  // -------------------------------------------------------------------------
  // 1. 输入检查与几何初值
  // -------------------------------------------------------------------------
  OmTrajectoryResult invalid;
  if (!initial.position.allFinite() || !terminal.position.allFinite() ||
    !initial.velocity.allFinite() || !terminal.velocity.allFinite())
  {
    invalid.status = "non-finite boundary state";
    return invalid;
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> path;
  path.push_back(initial.position);
  for (const auto &waypoint : waypoints) {
    if (!waypoint.position.allFinite() || !std::isfinite(waypoint.tolerance) ||
      waypoint.tolerance < 0.0)
    {
      invalid.status = "waypoint position/tolerance is invalid";
      return invalid;
    }
    path.push_back(waypoint.position);
  }
  if (waypoints.size() > static_cast<std::size_t>(options_.intervals - 1)) {
    invalid.status = "waypoint_count must be smaller than optimizer intervals";
    return invalid;
  }
  path.push_back(terminal.position);
  std::vector<double> cumulative(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    cumulative[i] = cumulative[i - 1] + (path[i] - path[i - 1]).norm();
  }
  const double length = std::max(cumulative.back(), 1.0e-3);
  // 折线只负责产生一个有序、接近可行域的初值，不是最终轨迹约束。进入 SCP 后，
  // 状态节点可以离开折线形成大弧线，航点通过时刻由 CSTC 进度变量重新选择。
  double candidate_time = std::clamp(
    // 初始时间公式 T0=clip(1.5*L/v_init,T_min,T_max)。1.5 是可行性裕度。
    1.5 * length / std::max(options_.initial_speed, 0.1),
    options_.minimum_time, options_.maximum_time);

  std::vector<OmTrajectoryIteration, Eigen::aligned_allocator<OmTrajectoryIteration>>
    complete_history;
  int time_attempt = 0;

  // The minimum-time objective is deliberately kept outside the QP.  Each QP
  // below is a fixed-T feasibility/restoration problem; therefore numerical
  // regularizers cannot trade a longer flight time for smoother control.

  const auto solve_fixed_time = [&](
      double total_time, const OmTrajectoryResult *warm_start,
      bool force_cstc_reselection) {
      // ---------------------------------------------------------------------
      // 2. 为当前候选总时间建立名义轨迹
      // ---------------------------------------------------------------------
      OmTrajectoryResult result;
      result.total_time = total_time;
      const int current_time_attempt = time_attempt++;
      const int n = options_.intervals;
      const int waypoint_count = static_cast<int>(waypoints.size());
      // Restoration uses virtual control and CSTC slack.  Do not silently add
      // actuator-rate or hover-endpoint constraints to the first solve: doing so
      // made initialization solve a smaller feasible set than the actual problem.
      const bool enforce_input_rates = options_.enforce_input_rate_constraints;
      const bool enforce_boundary_input = options_.enforce_hover_boundary_input;
      const bool use_cstc = options_.enable_cstc &&
        (force_cstc_reselection ||
        !(options_.lock_cstc_active_set_for_time_search && warm_start != nullptr));
      double nominal_time = total_time;
      double step = nominal_time / n;
      std::vector<int> waypoint_nodes;
      waypoint_nodes.reserve(waypoints.size());
      for (std::size_t j = 0; j < waypoints.size(); ++j) {
        int node = static_cast<int>(std::lround(n * cumulative[j + 1] / length));
        if (warm_start != nullptr && warm_start->success &&
          warm_start->total_time > 1.0e-9 &&
          warm_start->waypoint_times.size() == waypoints.size())
        {
          // Preserve the normalized CSTC passage location of the current feasible
          // incumbent when testing a shorter T.
          node = static_cast<int>(std::lround(
            n * warm_start->waypoint_times[j] / warm_start->total_time));
        }
        const int lower = waypoint_nodes.empty() ? 1 : waypoint_nodes.back() + 1;
        const int remaining = static_cast<int>(waypoints.size() - j);
        node = std::clamp(node, lower, n - remaining);
        waypoint_nodes.push_back(node);
      }

      std::vector<NominalState, Eigen::aligned_allocator<NominalState>> states(n + 1);
      const bool use_warm_start = warm_start != nullptr && warm_start->success &&
        warm_start->states.size() == static_cast<std::size_t>(n + 1) &&
        warm_start->total_time > 1.0e-9;
      if (use_warm_start) {
        // Time-dilation warm start. For p_new(s)=p_old(s), s=t/T,
        // v_new=(T_old/T_new)v_old. Attitude and input are reconstructed below
        // from the scaled acceleration so gravity is handled consistently.
        const double time_scale = warm_start->total_time / total_time;
        for (int k = 0; k <= n; ++k) {
          states[k].p = warm_start->states[static_cast<std::size_t>(k)].position;
          states[k].v = time_scale *
            warm_start->states[static_cast<std::size_t>(k)].velocity;
        }
      } else {
        for (int k = 0; k <= n; ++k) {
          const double distance = length * k / n;
          auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), distance);
          std::size_t segment = upper == cumulative.begin() ? 0 :
            static_cast<std::size_t>(upper - cumulative.begin() - 1);
          segment = std::min(segment, path.size() - 2);
          const double segment_length = std::max(
            cumulative[segment + 1] - cumulative[segment], 1.0e-9);
          const double alpha = std::clamp(
            (distance - cumulative[segment]) / segment_length, 0.0, 1.0);
          // Geometric fallback p_k=(1-alpha)p_i+alpha*p_{i+1}.
          states[k].p = (1.0 - alpha) * path[segment] + alpha * path[segment + 1];
        }
      }
      states.front().p = initial.position;
      states.back().p = terminal.position;
      if (!use_warm_start) {
        for (int k = 1; k < n; ++k) {
          // 中心差分初值 v_k=(p_{k+1}-p_{k-1})/(2*dt)。
          states[k].v = (states[k + 1].p - states[k - 1].p) / (2.0 * step);
        }
      }
      states.front().v = initial.velocity;
      states.back().v = terminal.velocity;
      const Eigen::Matrix3d r0 = initial.attitude.normalized().toRotationMatrix();
      const Eigen::Matrix3d rf = terminal.attitude.normalized().toRotationMatrix();
      const double yaw0 = yawOf(r0);
      const double yaw_delta = wrapAngle(yawOf(rf) - yaw0);
      for (int k = 0; k <= n; ++k) {
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
        if (use_warm_start && k < n) {
          // Match the forward-Euler velocity dynamics used by propagate():
          // a_k=(v_{k+1}-v_k)/dt, f_k=a_k+g e3.
          acceleration = (states[k + 1].v - states[k].v) / step;
        } else if (use_warm_start && n > 0) {
          acceleration = (states[k].v - states[k - 1].v) / step;
        } else if (k > 0 && k < n) {
          // The geometric fallback is less noisy with a centered acceleration.
          acceleration = (states[k + 1].v - states[k - 1].v) / (2.0 * step);
        }
        states[k].r = forceAttitude(
          acceleration + Eigen::Vector3d(0.0, 0.0, options_.gravity),
          yaw0 + yaw_delta * k / n);
      }
      states.front().r = r0;
      states.back().r = rf;
      std::vector<Eigen::Matrix<double, 4, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, 4, 1>>> inputs(n);
      for (int k = 0; k < n; ++k) {
        // 输入初值：a_T=(a+g*e3)^T R e3；omega=Log(R_k^T R_{k+1})/dt。
        Eigen::Vector3d acceleration = (states[k + 1].v - states[k].v) / step;
        inputs[k](0) = std::clamp(
          (acceleration + Eigen::Vector3d(0.0, 0.0, options_.gravity)).dot(states[k].r.col(2)),
          options_.thrust_acceleration_min, options_.thrust_acceleration_max);
        inputs[k].tail<3>() = logSo3(states[k].r.transpose() * states[k + 1].r) / step;
        inputs[k].tail<3>() = inputs[k].tail<3>().cwiseMax(-options_.body_rate_max)
          .cwiseMin(options_.body_rate_max);
      }

      // Fixed arc-length nodes are retained only as a dynamically sensible
      // one-hot initialization. CSTC is free to move progress to other nodes.
      // 对第 j 个点，lambda[k,j] 表示节点 k 时“尚未完成”的剩余进度，
      // mu[k,j] 是该步消耗的进度：lambda[k+1,j]=lambda[k,j]-mu[k,j]。
      // 边界 lambda[0,j]=1、lambda[N,j]=0 迫使每个航点恰好完成一次。
      Eigen::MatrixXd progress_lambda = Eigen::MatrixXd::Ones(n + 1, waypoint_count);
      Eigen::MatrixXd progress_mu = Eigen::MatrixXd::Zero(n, waypoint_count);
      const bool use_progress_warm_start = use_warm_start &&
        warm_start->progress_lambda.rows() == n + 1 &&
        warm_start->progress_lambda.cols() == waypoint_count &&
        warm_start->progress_mu.rows() == n &&
        warm_start->progress_mu.cols() == waypoint_count;
      if (use_progress_warm_start) {
        // Spatial path and normalized node locations are unchanged by time dilation,
        // so copying the complete CSTC variables preserves waypoint/order feasibility.
        progress_lambda = warm_start->progress_lambda;
        progress_mu = warm_start->progress_mu;
      } else {
        // A one-hot mu seed makes the linearized product mu*g degenerate:
        // wherever mu_bar=0 and g_bar>0, the QP obtains delta_mu=0 and no
        // position gradient.  Seed a compact, ordered distribution instead so
        // nearby nodes can jointly move in position and exchange progress.
        for (int j = 0; j < waypoint_count; ++j) {
          const int center = waypoint_nodes[static_cast<std::size_t>(j)];
          const int previous_center = j > 0 ?
            waypoint_nodes[static_cast<std::size_t>(j - 1)] : -1;
          const int next_center = j + 1 < waypoint_count ?
            waypoint_nodes[static_cast<std::size_t>(j + 1)] : n;
          int support_begin = std::max(0, center - options_.cstc_initial_support_radius);
          int support_end = std::min(n - 1, center + options_.cstc_initial_support_radius);
          if (j > 0) {
            support_begin = std::max(
              support_begin, (previous_center + center) / 2 + 1);
          }
          if (j + 1 < waypoint_count) {
            support_end = std::min(support_end, (center + next_center) / 2);
          }
          support_begin = std::min(support_begin, center);
          support_end = std::max(support_end, center);
          double weight_sum = 0.0;
          for (int k = support_begin; k <= support_end; ++k) {
            const double weight = static_cast<double>(
              options_.cstc_initial_support_radius + 1 - std::abs(k - center));
            progress_mu(k, j) = std::max(1.0, weight);
            weight_sum += progress_mu(k, j);
          }
          progress_mu.col(j) /= std::max(weight_sum, 1.0);
          progress_lambda(0, j) = 1.0;
          for (int k = 0; k < n; ++k) {
            progress_lambda(k + 1, j) =
              progress_lambda(k, j) - progress_mu(k, j);
          }
        }
      }

      // ---------------------------------------------------------------------
      // 3. QP 决策向量布局
      // ---------------------------------------------------------------------
      // z = [delta_x(0:N), delta_u(0:N-1), nu(0:N-1),
      //      delta_lambda, delta_mu, waypoint_slack, order_slack].
      // 前三类分别是流形状态增量、真实控制增量、动力学虚拟控制；最后一维
      // Total time is fixed in this inner problem and minimized by outer bisection.
      const int state_variables = 9 * (n + 1);
      const int input_variables = 4 * n;
      const int virtual_variables = 9 * n;
      const int lambda_variables = use_cstc ? (n + 1) * waypoint_count : 0;
      const int mu_variables = use_cstc ? n * waypoint_count : 0;
      const int waypoint_slack_variables = mu_variables;
      const int order_slack_variables = use_cstc && options_.enforce_strict_waypoint_order ?
        n * std::max(0, waypoint_count - 1) : 0;
      const int lambda_offset = state_variables + input_variables + virtual_variables;
      const int mu_offset = lambda_offset + lambda_variables;
      const int waypoint_slack_offset = mu_offset + mu_variables;
      const int order_slack_offset = waypoint_slack_offset + waypoint_slack_variables;
      const int variables = order_slack_offset + order_slack_variables;
      const auto x_index = [](int k, int component) {return 9 * k + component;};
      const auto u_index = [state_variables](int k, int component) {
          return state_variables + 4 * k + component;
        };
      const auto d_index = [state_variables, input_variables](int k, int component) {
          return state_variables + input_variables + 9 * k + component;
        };
      const auto lambda_index = [lambda_offset, waypoint_count](int k, int j) {
          return lambda_offset + k * waypoint_count + j;
        };
      const auto mu_index = [mu_offset, waypoint_count](int k, int j) {
          return mu_offset + k * waypoint_count + j;
        };
      const auto waypoint_slack_index = [waypoint_slack_offset, waypoint_count](int k, int j) {
          return waypoint_slack_offset + k * waypoint_count + j;
        };
      const auto order_slack_index = [order_slack_offset, waypoint_count](int k, int j) {
          return order_slack_offset + k * (waypoint_count - 1) + j;
        };

      double maximum_virtual = std::numeric_limits<double>::infinity();
      double maximum_nonlinear_defect = std::numeric_limits<double>::infinity();
      double maximum_waypoint_residual = std::numeric_limits<double>::infinity();
      double maximum_order_residual = std::numeric_limits<double>::infinity();
      double maximum_cstc_slack = std::numeric_limits<double>::infinity();
      double maximum_update = std::numeric_limits<double>::infinity();
      bool has_feasible_incumbent = false;
      double feasible_incumbent_time = std::numeric_limits<double>::infinity();
      double feasible_incumbent_merit = std::numeric_limits<double>::infinity();
      std::vector<NominalState, Eigen::aligned_allocator<NominalState>> feasible_states;
      std::vector<Eigen::Matrix<double, 4, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, 4, 1>>> feasible_inputs;
      Eigen::MatrixXd feasible_lambda;
      Eigen::MatrixXd feasible_mu;
      double feasible_dynamics = std::numeric_limits<double>::infinity();
      double feasible_virtual = std::numeric_limits<double>::infinity();
      double feasible_waypoint = std::numeric_limits<double>::infinity();
      double feasible_order = std::numeric_limits<double>::infinity();
      double feasible_slack = std::numeric_limits<double>::infinity();
      double feasible_update = std::numeric_limits<double>::infinity();
      bool scp_converged = false;
      std::string solver_status;
      double trust_scale = 1.0;
      for (int iteration = 0; iteration < options_.max_scp_iterations; ++iteration) {
        const auto iteration_wall_start = std::chrono::steady_clock::now();
        // Complementarity homotopy: early QPs may move progress through a small
        // relaxed product tube; the tube contracts geometrically to zero while
        // the final nonlinear feasibility check remains unchanged.
        const double cstc_relaxation = options_.cstc_relaxation_initial *
          std::pow(options_.cstc_relaxation_decay, iteration);
        // -------------------------------------------------------------------
        // 4. 在当前名义轨迹处线性化非线性离散动力学
        // -------------------------------------------------------------------
        std::vector<Matrix9, Eigen::aligned_allocator<Matrix9>> dynamics_a(n);
        std::vector<Matrix94, Eigen::aligned_allocator<Matrix94>> dynamics_b(n);
        std::vector<Vector9, Eigen::aligned_allocator<Vector9>> defects(n);
        for (int k = 0; k < n; ++k) {
          linearize(states[k], states[k + 1], inputs[k], step, options_.gravity,
            &dynamics_a[k], &dynamics_b[k], &defects[k]);
        }

        std::vector<Eigen::Triplet<double>> h_triplets;
        h_triplets.reserve(static_cast<std::size_t>(variables + 32 * n));
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variables);
        // -------------------------------------------------------------------
        // 5. 构造目标函数 Hessian 与梯度
        // -------------------------------------------------------------------
        // (F6) The inner fixed-T objective contains only numerical SCP
        // regularization, feasibility restoration, and CSTC slack penalties.
        // Actual-control terms are only fixed-T tie-breakers. Since T is an outer
        // lexicographic variable, they cannot purchase a longer flight time.
        // Expanding control smoothness produces
        // Hessian 的相邻块和由名义控制差引起的一次项。
        // 完整实现目标为
        // J = rho_x sum||dx_k||^2
        //   + rho_u sum||u_bar_k+du_k-u_hover||^2
        //   + rho_Du sum||(u_bar_k+du_k)-(u_bar_{k-1}+du_{k-1})||^2
        //   + rho_nu sum||nu_k||^2 + rho_mu sum||dmu_k||^2
        //   + w_s sum(s_wp+s_order).
        for (int i = 0; i < state_variables; ++i) {
          h_triplets.emplace_back(i, i, 2.0 * options_.state_regularization);
        }
        for (int k = 0; k < n; ++k) {
          for (int j = 0; j < 4; ++j) {
            const int index = u_index(k, j);
            const double input_reference = j == 0 ? options_.gravity : 0.0;
            const double input_error = inputs[k](j) - input_reference;
            h_triplets.emplace_back(index, index, 2.0 * options_.input_regularization);
            gradient(index) += 2.0 * options_.input_regularization * input_error;
            if (k > 0) {
              const int previous = u_index(k - 1, j);
              const double w = 2.0 * options_.input_smoothness;
              const double nominal_difference = inputs[k](j) - inputs[k - 1](j);
              h_triplets.emplace_back(index, index, w);
              h_triplets.emplace_back(previous, previous, w);
              h_triplets.emplace_back(previous, index, -w);
              h_triplets.emplace_back(index, previous, -w);
              gradient(index) += 2.0 * options_.input_smoothness * nominal_difference;
              gradient(previous) -= 2.0 * options_.input_smoothness * nominal_difference;
            }
          }
          for (int j = 0; j < 9; ++j) {
            h_triplets.emplace_back(d_index(k, j), d_index(k, j),
              2.0 * options_.virtual_control_weight);
          }
        }
        if (use_cstc) {
          for (int k = 0; k <= n; ++k) {
            for (int j = 0; j < waypoint_count; ++j) {
              h_triplets.emplace_back(lambda_index(k, j), lambda_index(k, j), 2.0e-8);
            }
          }
          for (int k = 0; k < n; ++k) {
            for (int j = 0; j < waypoint_count; ++j) {
              h_triplets.emplace_back(mu_index(k, j), mu_index(k, j),
                2.0 * options_.progress_regularization);
              h_triplets.emplace_back(waypoint_slack_index(k, j),
                waypoint_slack_index(k, j), 2.0e-8);
            }
            for (int j = 0; j + 1 < waypoint_count &&
              options_.enforce_strict_waypoint_order; ++j)
            {
              h_triplets.emplace_back(order_slack_index(k, j),
                order_slack_index(k, j), 2.0e-8);
            }
          }
        }
        Eigen::SparseMatrix<double> hessian(variables, variables);
        hessian.setFromTriplets(h_triplets.begin(), h_triplets.end());
        if (use_cstc) {
          for (int k = 0; k < n; ++k) {
            for (int j = 0; j < waypoint_count; ++j) {
              gradient(waypoint_slack_index(k, j)) = options_.cstc_slack_weight;
            }
            for (int j = 0; j + 1 < waypoint_count &&
              options_.enforce_strict_waypoint_order; ++j)
            {
              gradient(order_slack_index(k, j)) = options_.cstc_slack_weight;
            }
          }
        }

        // -------------------------------------------------------------------
        // 6. 预计算约束行数并组装 l <= A*z <= u
        // -------------------------------------------------------------------
        const int equality_rows = 18 + 9 * n;
        const int progress_rows = use_cstc ? n * waypoint_count : 0;
        const int progress_boundary_rows = use_cstc ? 2 * waypoint_count : 0;
        const int weak_order_rows = use_cstc ?
          (n + 1) * std::max(0, waypoint_count - 1) : 0;
        const int waypoint_cstc_rows = use_cstc ? n * waypoint_count : 0;
        const int strict_order_rows = use_cstc && options_.enforce_strict_waypoint_order ?
          n * std::max(0, waypoint_count - 1) : 0;
        const int fixed_waypoint_rows = use_cstc ? 0 : 3 * waypoint_count;
        const int rate_transitions = enforce_input_rates ?
          (n - 1 + (enforce_boundary_input ? 2 : 0)) : 0;
        const int input_rate_rows = 2 * 4 * rate_transitions;
        const int rows = equality_rows + progress_rows + progress_boundary_rows +
          weak_order_rows + waypoint_cstc_rows + strict_order_rows +
          fixed_waypoint_rows + input_rate_rows + variables;
        std::vector<Eigen::Triplet<double>> a_triplets;
        Eigen::VectorXd lower = Eigen::VectorXd::Constant(rows, -OSQP_INFTY);
        Eigen::VectorXd upper = Eigen::VectorXd::Constant(rows, OSQP_INFTY);
        int row = 0;
        // 初末状态的 18 个等式：初始增量为零，末端增量消除当前末端误差。
        for (int j = 0; j < 9; ++j, ++row) {
          a_triplets.emplace_back(row, x_index(0, j), 1.0);
          lower(row) = upper(row) = 0.0;
        }
        NominalState terminal_state;
        terminal_state.p = terminal.position;
        terminal_state.v = terminal.velocity;
        terminal_state.r = rf;
        const Vector9 terminal_error = difference(terminal_state, states.back());
        for (int j = 0; j < 9; ++j, ++row) {
          a_triplets.emplace_back(row, x_index(n, j), 1.0);
          lower(row) = upper(row) = terminal_error(j);
        }
        for (int k = 0; k < n; ++k) {
          // Fixed-T linearized dynamics equality.
          for (int j = 0; j < 9; ++j, ++row) {
            a_triplets.emplace_back(row, x_index(k + 1, j), 1.0);
            a_triplets.emplace_back(row, d_index(k, j), -1.0);
            for (int c = 0; c < 9; ++c) {
              if (std::abs(dynamics_a[k](j, c)) > 1.0e-12) {
                a_triplets.emplace_back(row, x_index(k, c), -dynamics_a[k](j, c));
              }
            }
            for (int c = 0; c < 4; ++c) {
              if (std::abs(dynamics_b[k](j, c)) > 1.0e-12) {
                a_triplets.emplace_back(row, u_index(k, c), -dynamics_b[k](j, c));
              }
            }
            lower(row) = upper(row) = defects[k](j);
          }
        }
        if (use_cstc) {
          // (F7) 精确仿射进度动力学及边界：lambda_0=1, lambda_N=0，
          // lambda_{k+1}=lambda_k-mu_k。
          for (int k = 0; k < n; ++k) {
            for (int j = 0; j < waypoint_count; ++j, ++row) {
              a_triplets.emplace_back(row, lambda_index(k + 1, j), 1.0);
              a_triplets.emplace_back(row, lambda_index(k, j), -1.0);
              a_triplets.emplace_back(row, mu_index(k, j), 1.0);
              const double residual = progress_lambda(k, j) - progress_mu(k, j) -
                progress_lambda(k + 1, j);
              lower(row) = upper(row) = residual;
            }
          }
          for (int j = 0; j < waypoint_count; ++j) {
            a_triplets.emplace_back(row, lambda_index(0, j), 1.0);
            lower(row) = upper(row) = 1.0 - progress_lambda(0, j);
            ++row;
            a_triplets.emplace_back(row, lambda_index(n, j), 1.0);
            lower(row) = upper(row) = -progress_lambda(n, j);
            ++row;
          }
          // 弱次序：lambda_{k,j}<=lambda_{k,j+1}，即前一航点的剩余进度
          // 不得大于后一航点，防止完成顺序逆转。
          for (int k = 0; k <= n; ++k) {
            for (int j = 0; j + 1 < waypoint_count; ++j, ++row) {
              a_triplets.emplace_back(row, lambda_index(k, j), 1.0);
              a_triplets.emplace_back(row, lambda_index(k, j + 1), -1.0);
              upper(row) = progress_lambda(k, j + 1) - progress_lambda(k, j);
            }
          }
          // (F8) 空间互补约束的目标原式为
          //   mu[k,j] * (||p[k]-p_j||^2-r_j^2) <= 0.
          // SCP 前期使用右端 sigma_i+s，sigma_i 几何衰减至 0，以避免
          // mu_bar=0 时位置梯度与进度增量同时锁死；最终可行性仍按原式检查。
          // 因 mu>=0，当该步消耗进度(mu>0)时，位置必须进入航点球；当飞机
          // 不在航点球内时只能令 mu=0。下面对乘积在名义点作一阶展开。
          for (int k = 0; k < n; ++k) {
            for (int j = 0; j < waypoint_count; ++j, ++row) {
              const Eigen::Vector3d position_error = states[k].p - waypoints[j].position;
              const double distance_residual = position_error.squaredNorm() -
                waypoints[j].tolerance * waypoints[j].tolerance;
              for (int c = 0; c < 3; ++c) {
                const double coefficient = 2.0 * progress_mu(k, j) * position_error(c);
                if (std::abs(coefficient) > 1.0e-12) {
                  a_triplets.emplace_back(row, x_index(k, c), coefficient);
                }
              }
              a_triplets.emplace_back(row, mu_index(k, j), distance_residual);
              a_triplets.emplace_back(row, waypoint_slack_index(k, j), -1.0);
              upper(row) = cstc_relaxation -
                progress_mu(k, j) * distance_residual;
            }
          }
          // (F9) 严格次序互补的目标是 lambda[k,j]*mu[k,j+1]=0；同样先用
          // sigma_i+s 延拓到原式。只要前一点仍有剩余进度，
          // 后一点就不能开始消耗进度；结合弱次序约束可防止多航点互换顺序。
          if (options_.enforce_strict_waypoint_order) {
            for (int k = 0; k < n; ++k) {
              for (int j = 0; j + 1 < waypoint_count; ++j, ++row) {
                a_triplets.emplace_back(row, lambda_index(k, j), progress_mu(k, j + 1));
                a_triplets.emplace_back(row, mu_index(k, j + 1), progress_lambda(k, j));
                a_triplets.emplace_back(row, order_slack_index(k, j), -1.0);
                upper(row) = cstc_relaxation -
                  progress_lambda(k, j) * progress_mu(k, j + 1);
              }
            }
          }
        } else {
          for (int j = 0; j < waypoint_count; ++j) {
            const int node = waypoint_nodes[static_cast<std::size_t>(j)];
            const double component_tolerance =
              std::max(0.0, waypoints[j].tolerance) / std::sqrt(3.0);
            const Eigen::Vector3d correction = waypoints[j].position - states[node].p;
            for (int c = 0; c < 3; ++c, ++row) {
              a_triplets.emplace_back(row, x_index(node, c), 1.0);
              lower(row) = correction(c) - component_tolerance;
              upper(row) = correction(c) + component_tolerance;
            }
          }
        }
        if (enforce_input_rates) {
          // (F10) Optional actuator-bandwidth constraint on the actual input:
          // |u_k-u_{k-1}| <= rate_max*(T/N). It is disabled in pure paper mode.
          const auto rate_limit = [&](int component) {
              return component == 0 ? options_.thrust_acceleration_rate_max :
                options_.body_rate_acceleration_max(component - 1);
            };
          const auto add_rate_rows = [&](int current_index, double current_coefficient,
              int previous_index, double previous_coefficient,
              double nominal_difference, double rate) {
              a_triplets.emplace_back(row, current_index, current_coefficient);
              if (previous_index >= 0) {
                a_triplets.emplace_back(row, previous_index, previous_coefficient);
              }
              upper(row) = rate * step - nominal_difference;
              ++row;

              a_triplets.emplace_back(row, current_index, current_coefficient);
              if (previous_index >= 0) {
                a_triplets.emplace_back(row, previous_index, previous_coefficient);
              }
              lower(row) = -rate * step - nominal_difference;
              ++row;
            };
          for (int k = 1; k < n; ++k) {
            for (int component = 0; component < 4; ++component) {
              add_rate_rows(
                u_index(k, component), 1.0, u_index(k - 1, component), -1.0,
                inputs[k](component) - inputs[k - 1](component), rate_limit(component));
            }
          }
          if (enforce_boundary_input) {
            for (int component = 0; component < 4; ++component) {
              const double hover = component == 0 ? options_.gravity : 0.0;
              add_rate_rows(
                u_index(0, component), 1.0, -1, 0.0,
                inputs[0](component) - hover, rate_limit(component));
              add_rate_rows(
                u_index(n - 1, component), -1.0, -1, 0.0,
                hover - inputs[n - 1](component), rate_limit(component));
            }
          }
        }
        for (int variable = 0; variable < variables; ++variable, ++row) {
          // (F11) 单位矩阵行统一实现信赖域、真实输入上下界、进度变量 [0,1]、
          // 非负松弛，以及总时间增量的信赖域。
          a_triplets.emplace_back(row, variable, 1.0);
          if (variable < state_variables) {
            const int component = variable % 9;
            const double trust = component < 3 ? options_.position_trust_region :
              (component < 6 ? options_.velocity_trust_region : options_.attitude_trust_region);
            lower(row) = -trust_scale * trust;
            upper(row) = trust_scale * trust;
          } else if (variable < state_variables + input_variables) {
            const int local = variable - state_variables;
            const int k = local / 4;
            const int component = local % 4;
            if (component == 0) {
              lower(row) = std::max(
                options_.thrust_acceleration_min - inputs[k](0),
                -trust_scale * options_.thrust_trust_region);
              upper(row) = std::min(
                options_.thrust_acceleration_max - inputs[k](0),
                trust_scale * options_.thrust_trust_region);
            } else {
              const double input_trust =
                trust_scale * options_.body_rate_trust_region(component - 1);
              lower(row) = std::max(
                -options_.body_rate_max(component - 1) - inputs[k](component), -input_trust);
              upper(row) = std::min(
                options_.body_rate_max(component - 1) - inputs[k](component), input_trust);
            }
          } else if (variable >= lambda_offset && variable < mu_offset) {
            const int local = variable - lambda_offset;
            const int k = local / waypoint_count;
            const int j = local % waypoint_count;
            if (use_progress_warm_start &&
              iteration < options_.cstc_warm_start_iterations)
            {
              // Keep a previously feasible passage schedule fixed while the first
              // three SCP steps restore dynamics after time dilation.
              lower(row) = upper(row) = 0.0;
            } else {
              const double progress_trust = trust_scale * options_.progress_trust_region;
              lower(row) = std::max(-progress_trust, -progress_lambda(k, j));
              upper(row) = std::min(progress_trust, 1.0 - progress_lambda(k, j));
            }
          } else if (variable >= mu_offset && variable < waypoint_slack_offset) {
            const int local = variable - mu_offset;
            const int k = local / waypoint_count;
            const int j = local % waypoint_count;
            if (use_progress_warm_start &&
              iteration < options_.cstc_warm_start_iterations)
            {
              lower(row) = upper(row) = 0.0;
            } else {
              const double progress_trust = trust_scale * options_.progress_trust_region;
              lower(row) = std::max(-progress_trust, -progress_mu(k, j));
              upper(row) = std::min(progress_trust, 1.0 - progress_mu(k, j));
            }
          } else if (variable >= waypoint_slack_offset) {
            lower(row) = 0.0;
          }
        }
        if (row != rows) {
          result.status = "internal constraint row mismatch";
          return result;
        }
        Eigen::SparseMatrix<double> constraint(rows, variables);
        constraint.setFromTriplets(a_triplets.begin(), a_triplets.end());
        Eigen::VectorXd solution;
        const auto qp_wall_start = std::chrono::steady_clock::now();
        const bool qp_solved = solveOsqp(
          hessian, gradient, constraint, lower, upper, &solution, &solver_status);
        const double qp_solve_time = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - qp_wall_start).count();
        if (!qp_solved) {
          // A rejected QP is not inserted into the RViz trajectory history, but
          // is reported live so time-trust-region retries are still observable.
          if (options_.progress_callback) {
            OmTrajectoryIteration rejected;
            rejected.time_attempt = current_time_attempt;
            rejected.scp_iteration = iteration + 1;
            rejected.total_time = nominal_time;
            rejected.step_solve_time = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - iteration_wall_start).count();
            rejected.qp_solve_time = qp_solve_time;
            rejected.elapsed_solve_time = elapsed_solve_time();
            rejected.solver_status = "REJECTED: " + solver_status;
            options_.progress_callback(rejected);
          }
          result.status = solver_status;
          return result;
        }
        // -------------------------------------------------------------------
        // 7. Nonlinear backtracking and adaptive trust-region acceptance.
        // A solved convex subproblem is only a proposed step.  Evaluate the
        // original nonlinear dynamics/complementarity for alpha=1,1/2,... and
        // keep the best candidate instead of unconditionally taking alpha=1.
        // -------------------------------------------------------------------
        const auto evaluate_nonlinear = [&] (
            const std::vector<NominalState, Eigen::aligned_allocator<NominalState>> &xs,
            const std::vector<Eigen::Matrix<double, 4, 1>,
              Eigen::aligned_allocator<Eigen::Matrix<double, 4, 1>>> &us,
            const Eigen::MatrixXd &lambda, const Eigen::MatrixXd &mu,
            double *dynamics, double *waypoint, double *order) {
            *dynamics = 0.0;
            *waypoint = 0.0;
            *order = 0.0;
            for (int k = 0; k < n; ++k) {
              *dynamics = std::max(*dynamics,
                difference(propagate(xs[k], us[k], step, options_.gravity),
                xs[k + 1]).lpNorm<Eigen::Infinity>());
            }
            if (use_cstc) {
              for (int k = 0; k < n; ++k) {
                for (int j = 0; j < waypoint_count; ++j) {
                  const double outside = std::max(0.0,
                    (xs[k].p - waypoints[j].position).squaredNorm() -
                    waypoints[j].tolerance * waypoints[j].tolerance);
                  *waypoint = std::max(*waypoint, mu(k, j) * outside);
                }
                for (int j = 0; j + 1 < waypoint_count; ++j) {
                  *order = std::max(*order, lambda(k, j) * mu(k, j + 1));
                }
              }
            } else {
              // Fixed active-set rows use an inscribed box. Recheck the original
              // waypoint sphere so a damped step cannot be reported as feasible
              // merely because CSTC variables are absent.
              for (int j = 0; j < waypoint_count; ++j) {
                const int node = waypoint_nodes[static_cast<std::size_t>(j)];
                *waypoint = std::max(*waypoint, std::max(0.0,
                  (xs[node].p - waypoints[j].position).squaredNorm() -
                  waypoints[j].tolerance * waypoints[j].tolerance));
              }
            }
          };

        double current_dynamics = 0.0;
        double current_waypoint = 0.0;
        double current_order = 0.0;
        evaluate_nonlinear(states, inputs, progress_lambda, progress_mu,
          &current_dynamics, &current_waypoint, &current_order);
        const auto relaxed_merit = [&](double dynamics, double waypoint, double order) {
            return dynamics / std::max(options_.dynamics_tolerance, 1.0e-12) +
              std::max(0.0, waypoint - cstc_relaxation) /
              std::max(options_.cstc_tolerance, 1.0e-12) +
              std::max(0.0, order - cstc_relaxation) /
              std::max(options_.cstc_tolerance, 1.0e-12);
          };
        const double current_merit = relaxed_merit(
          current_dynamics, current_waypoint, current_order);
        double best_step_merit = std::numeric_limits<double>::infinity();
        double accepted_alpha = 0.0;
        double accepted_dynamics = current_dynamics;
        double accepted_waypoint = current_waypoint;
        double accepted_order = current_order;
        auto accepted_states = states;
        auto accepted_inputs = inputs;
        Eigen::MatrixXd accepted_lambda = progress_lambda;
        Eigen::MatrixXd accepted_mu = progress_mu;

        for (int backtrack = 0; backtrack < options_.scp_backtracking_steps; ++backtrack) {
          const double alpha = std::ldexp(1.0, -backtrack);
          auto trial_states = states;
          auto trial_inputs = inputs;
          Eigen::MatrixXd trial_lambda = progress_lambda;
          Eigen::MatrixXd trial_mu = progress_mu;
          for (int k = 0; k <= n; ++k) {
            const Vector9 update = alpha * solution.segment<9>(x_index(k, 0));
            trial_states[k].p += update.segment<3>(0);
            trial_states[k].v += update.segment<3>(3);
            trial_states[k].r = trial_states[k].r * expSo3(update.segment<3>(6));
          }
          for (int k = 0; k < n; ++k) {
            trial_inputs[k] += alpha * solution.segment<4>(u_index(k, 0));
          }
          if (use_cstc) {
            for (int k = 0; k <= n; ++k) {
              for (int j = 0; j < waypoint_count; ++j) {
                trial_lambda(k, j) += alpha * solution(lambda_index(k, j));
              }
            }
            for (int k = 0; k < n; ++k) {
              for (int j = 0; j < waypoint_count; ++j) {
                trial_mu(k, j) += alpha * solution(mu_index(k, j));
              }
            }
          }
          double trial_dynamics = 0.0;
          double trial_waypoint = 0.0;
          double trial_order = 0.0;
          evaluate_nonlinear(trial_states, trial_inputs, trial_lambda, trial_mu,
            &trial_dynamics, &trial_waypoint, &trial_order);
          const double trial_merit = relaxed_merit(
            trial_dynamics, trial_waypoint, trial_order);
          if (trial_merit < best_step_merit) {
            best_step_merit = trial_merit;
            accepted_alpha = alpha;
            accepted_dynamics = trial_dynamics;
            accepted_waypoint = trial_waypoint;
            accepted_order = trial_order;
            accepted_states = std::move(trial_states);
            accepted_inputs = std::move(trial_inputs);
            accepted_lambda = std::move(trial_lambda);
            accepted_mu = std::move(trial_mu);
          }
        }

        // Reject a step that does not improve the relaxed nonlinear model. The
        // next SCP iteration rebuilds the QP with a smaller trust region.
        if (!(best_step_merit <= current_merit + 1.0e-9)) {
          trust_scale = std::max(0.05, 0.5 * trust_scale);
          solver_status += " | nonlinear step rejected, trust=" +
            std::to_string(trust_scale);
          continue;
        }
        states = std::move(accepted_states);
        inputs = std::move(accepted_inputs);
        progress_lambda = std::move(accepted_lambda);
        progress_mu = std::move(accepted_mu);
        maximum_nonlinear_defect = accepted_dynamics;
        maximum_waypoint_residual = accepted_waypoint;
        maximum_order_residual = accepted_order;
        maximum_update = 0.0;
        maximum_virtual = 0.0;
        for (int k = 0; k <= n; ++k) {
          maximum_update = std::max(maximum_update,
            accepted_alpha * solution.segment<9>(x_index(k, 0)).lpNorm<Eigen::Infinity>());
        }
        for (int k = 0; k < n; ++k) {
          maximum_virtual = std::max(maximum_virtual,
            accepted_alpha * solution.segment<9>(d_index(k, 0)).lpNorm<Eigen::Infinity>());
        }
        maximum_cstc_slack = 0.0;
        if (use_cstc) {
          for (int k = 0; k < n; ++k) {
            for (int j = 0; j < waypoint_count; ++j) {
              maximum_cstc_slack = std::max(maximum_cstc_slack,
                accepted_alpha * solution(waypoint_slack_index(k, j)));
            }
            for (int j = 0; j + 1 < waypoint_count &&
              options_.enforce_strict_waypoint_order; ++j)
            {
              maximum_cstc_slack = std::max(maximum_cstc_slack,
                accepted_alpha * solution(order_slack_index(k, j)));
            }
          }
        }
        // Repeated damping indicates that the local convex model is too broad;
        // contract its next trust region. Full accepted steps expand it again.
        trust_scale = accepted_alpha >= 0.999 ?
          std::min(1.0, 1.25 * trust_scale) : std::max(0.05, 0.8 * trust_scale);
        solver_status += " | alpha=" + std::to_string(accepted_alpha) +
          ", trust=" + std::to_string(trust_scale);
        const bool residuals_feasible =
          maximum_virtual < options_.dynamics_tolerance &&
          maximum_nonlinear_defect < options_.dynamics_tolerance &&
          maximum_waypoint_residual < options_.cstc_tolerance &&
          maximum_order_residual < options_.cstc_tolerance &&
          maximum_cstc_slack < options_.cstc_slack_tolerance;
        const double feasibility_merit =
          maximum_nonlinear_defect / std::max(options_.dynamics_tolerance, 1.0e-12) +
          maximum_virtual / std::max(options_.dynamics_tolerance, 1.0e-12) +
          maximum_waypoint_residual / std::max(options_.cstc_tolerance, 1.0e-12) +
          maximum_order_residual / std::max(options_.cstc_tolerance, 1.0e-12) +
          maximum_cstc_slack / std::max(options_.cstc_slack_tolerance, 1.0e-12);
        const bool shorter_feasible = nominal_time < feasible_incumbent_time - 1.0e-9;
        const bool better_same_time =
          std::abs(nominal_time - feasible_incumbent_time) <= 1.0e-9 &&
          feasibility_merit < feasible_incumbent_merit;
        if (residuals_feasible &&
          (!has_feasible_incumbent || shorter_feasible || better_same_time))
        {
          // At fixed T keep the feasible nonlinear iterate with the best normalized
          // residual merit. This avoids returning the first feasible SCP iterate.
          has_feasible_incumbent = true;
          feasible_incumbent_time = nominal_time;
          feasible_incumbent_merit = feasibility_merit;
          feasible_states = states;
          feasible_inputs = inputs;
          feasible_lambda = progress_lambda;
          feasible_mu = progress_mu;
          feasible_dynamics = maximum_nonlinear_defect;
          feasible_virtual = maximum_virtual;
          feasible_waypoint = maximum_waypoint_residual;
          feasible_order = maximum_order_residual;
          feasible_slack = maximum_cstc_slack;
          feasible_update = maximum_update;
        }

        OmTrajectoryIteration history_entry;
        // 每次 SCP 都保存状态、输入和残差，RViz 按 history 顺序播放收敛过程。
        history_entry.time_attempt = current_time_attempt;
        history_entry.scp_iteration = iteration + 1;
        history_entry.total_time = nominal_time;
        history_entry.maximum_update = maximum_update;
        history_entry.maximum_dynamics_defect = maximum_nonlinear_defect;
        history_entry.maximum_virtual_control = maximum_virtual;
        history_entry.maximum_waypoint_residual = maximum_waypoint_residual;
        history_entry.maximum_order_residual = maximum_order_residual;
        history_entry.maximum_cstc_slack = maximum_cstc_slack;
        history_entry.step_solve_time = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - iteration_wall_start).count();
        history_entry.qp_solve_time = qp_solve_time;
        history_entry.elapsed_solve_time = elapsed_solve_time();
        history_entry.solver_status = solver_status;
        history_entry.states.resize(static_cast<std::size_t>(n + 1));
        for (int k = 0; k <= n; ++k) {
          auto &output = history_entry.states[static_cast<std::size_t>(k)];
          output.time = k * step;
          output.position = states[k].p;
          output.velocity = states[k].v;
          output.attitude = Eigen::Quaterniond(states[k].r).normalized();
          const int input_index = std::min(k, n - 1);
          output.thrust_acceleration = inputs[input_index](0);
          output.body_rate = inputs[input_index].tail<3>();
        }
        if (options_.progress_callback) {
          options_.progress_callback(history_entry);
        }
        complete_history.push_back(std::move(history_entry));
        result.scp_iterations = iteration + 1;
        if (maximum_update < options_.convergence_tolerance && residuals_feasible)
        {
          scp_converged = true;
          break;
        }
      }
      if (has_feasible_incumbent) {
        // 联合最短时间模式返回整个 SCP 过程中记录到的最短非线性可行解。
        nominal_time = feasible_incumbent_time;
        step = nominal_time / n;
        states = feasible_states;
        inputs = feasible_inputs;
        progress_lambda = feasible_lambda;
        progress_mu = feasible_mu;
        maximum_nonlinear_defect = feasible_dynamics;
        maximum_virtual = feasible_virtual;
        maximum_waypoint_residual = feasible_waypoint;
        maximum_order_residual = feasible_order;
        maximum_cstc_slack = feasible_slack;
        maximum_update = feasible_update;
      }
      result.total_time = nominal_time;
      result.maximum_update = maximum_update;
      result.maximum_dynamics_defect = maximum_nonlinear_defect;
      result.maximum_virtual_control = maximum_virtual;
      result.maximum_waypoint_residual = maximum_waypoint_residual;
      result.maximum_order_residual = maximum_order_residual;
      result.maximum_cstc_slack = maximum_cstc_slack;
      result.success = std::isfinite(maximum_nonlinear_defect) &&
        maximum_virtual <= options_.dynamics_tolerance &&
        maximum_nonlinear_defect <= options_.dynamics_tolerance &&
        maximum_waypoint_residual <= options_.cstc_tolerance &&
        maximum_order_residual <= options_.cstc_tolerance &&
        maximum_cstc_slack <= options_.cstc_slack_tolerance;
      result.converged = result.success && scp_converged &&
        maximum_update <= options_.convergence_tolerance;
      result.status = result.converged ? "solved and converged" :
        (result.success ? "constraint-feasible but SCP not converged" :
        "SCP residuals: update=" + std::to_string(maximum_update) +
        ", dynamics=" + std::to_string(maximum_nonlinear_defect) +
        ", virtual=" + std::to_string(maximum_virtual) +
        ", waypoint=" + std::to_string(maximum_waypoint_residual) +
        ", order=" + std::to_string(maximum_order_residual) +
        ", slack=" + std::to_string(maximum_cstc_slack));
      if (result.success) {
        // (F13) 将内部状态转换成结果，并由 mu 重心恢复通过时刻：
        // t_j=(T/N)*sum_k(k*mu_{k,j})。
        result.states.resize(static_cast<std::size_t>(n + 1));
        for (int k = 0; k <= n; ++k) {
          auto &output = result.states[static_cast<std::size_t>(k)];
          output.time = k * step;
          output.position = states[k].p;
          output.velocity = states[k].v;
          output.attitude = Eigen::Quaterniond(states[k].r).normalized();
          const int input_index = std::min(k, n - 1);
          output.thrust_acceleration = inputs[input_index](0);
          output.body_rate = inputs[input_index].tail<3>();
        }
        result.waypoint_times.resize(static_cast<std::size_t>(waypoint_count));
        result.progress_lambda = progress_lambda;
        result.progress_mu = progress_mu;
        for (int j = 0; j < waypoint_count; ++j) {
          double selected_node = waypoint_nodes[static_cast<std::size_t>(j)];
          if (use_cstc) {
            selected_node = 0.0;
            for (int k = 0; k < n; ++k) {
              selected_node += k * progress_mu(k, j);
            }
          }
          result.waypoint_times[static_cast<std::size_t>(j)] = step * selected_node;
        }
        OmTrajectoryIteration final_history;
        final_history.time_attempt = current_time_attempt;
        final_history.scp_iteration = result.scp_iterations;
        final_history.total_time = nominal_time;
        final_history.maximum_update = maximum_update;
        final_history.maximum_dynamics_defect = maximum_nonlinear_defect;
        final_history.maximum_virtual_control = maximum_virtual;
        final_history.maximum_waypoint_residual = maximum_waypoint_residual;
        final_history.maximum_order_residual = maximum_order_residual;
        final_history.maximum_cstc_slack = maximum_cstc_slack;
        if (!complete_history.empty()) {
          final_history.step_solve_time = complete_history.back().step_solve_time;
          final_history.qp_solve_time = complete_history.back().qp_solve_time;
          final_history.elapsed_solve_time = complete_history.back().elapsed_solve_time;
          final_history.solver_status = complete_history.back().solver_status;
        }
        final_history.states = result.states;
        complete_history.push_back(std::move(final_history));
      }
      return result;
    };

  // -------------------------------------------------------------------------
  // 8. 可行时间初始化与最短时间策略
  // -------------------------------------------------------------------------
  // (F12) Strict lexicographic minimum time:
  //   T* = inf { T | F(T) is nonempty },
  // where F(T) contains the nonlinear dynamics, boundary conditions, input-box
  // bounds and ordered CSTC waypoint constraints.  The inner OSQP objective never
  // contains a trajectory-performance term, so it cannot buy smoothness with time.
  // First obtain a feasible upper bound, expanding the geometric estimate if needed.
  OmTrajectoryResult best = solve_fixed_time(candidate_time, nullptr, false);
  while (!best.success && candidate_time < options_.maximum_time - 1.0e-9) {
    candidate_time = std::min(options_.maximum_time, 1.35 * candidate_time);
    best = solve_fixed_time(candidate_time, nullptr, false);
  }
  if (!best.success) {
    best.history = complete_history;
    best.total_solve_time = elapsed_solve_time();
    return best;
  }
  if (!options_.optimize_total_time) {
    best.status = "fixed-time feasibility " + best.status;
    best.history = complete_history;
    best.total_solve_time = elapsed_solve_time();
    return best;
  }

  double feasible_time = candidate_time;
  double infeasible_time = options_.minimum_time;

  // Do not merely assume Tmin is infeasible: if it is feasible it is the global
  // optimum permitted by the configured time bound.
  if (options_.minimum_time < feasible_time - 1.0e-9) {
    OmTrajectoryResult minimum_trial = solve_fixed_time(options_.minimum_time, &best, false);
    if (!minimum_trial.success && options_.enable_cstc &&
      options_.lock_cstc_active_set_for_time_search &&
      options_.retry_cstc_on_locked_failure)
    {
      // A locked-node failure is a numerical/local-mode failure, not proof that
      // this flight time is physically infeasible. Reopen CSTC once.
      minimum_trial = solve_fixed_time(options_.minimum_time, &best, true);
    }
    if (minimum_trial.success) {
      best = std::move(minimum_trial);
      feasible_time = options_.minimum_time;
    } else {
      for (int search = 0; search < options_.max_time_search_iterations; ++search) {
        if (feasible_time - infeasible_time <= options_.time_search_tolerance) {
          break;
        }
        const double trial_time = 0.5 * (feasible_time + infeasible_time);
        OmTrajectoryResult trial = solve_fixed_time(trial_time, &best, false);
        if (!trial.success && options_.enable_cstc &&
          options_.lock_cstc_active_set_for_time_search &&
          options_.retry_cstc_on_locked_failure)
        {
          trial = solve_fixed_time(trial_time, &best, true);
        }
        if (trial.success) {
          best = std::move(trial);
          feasible_time = trial_time;
        } else {
          infeasible_time = trial_time;
        }
      }
    }
  }
  const std::string minimum_time_method =
    options_.enable_cstc && options_.lock_cstc_active_set_for_time_search ?
    "CSTC-active-set pure-minimum-time bisection" :
    "pure-minimum-time bisection";
  best.status = minimum_time_method + ", bracket=[" +
    std::to_string(infeasible_time) + ", " + std::to_string(feasible_time) + "] s; " +
    best.status;
  best.history = complete_history;
  best.total_solve_time = elapsed_solve_time();
  return best;
}

bool saveOmTrajectoryCsv(
  const OmTrajectoryResult &trajectory, const std::string &file_path,
  std::string *error)
{
  const auto fail = [error](const std::string &message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
  if (!trajectory.success || trajectory.states.size() < 2U || file_path.empty()) {
    return fail("trajectory is unsuccessful/empty or output path is empty");
  }

  const std::string temporary_path = file_path + ".tmp";
  std::ofstream stream(temporary_path, std::ios::out | std::ios::trunc);
  if (!stream.is_open()) {
    return fail("cannot open temporary output file: " + temporary_path);
  }
  stream << "# px4ctrl_omtraj_v1\n"
         << "# time,px,py,pz,vx,vy,vz,qw,qx,qy,qz,thrust_acceleration,wx,wy,wz\n"
         << std::setprecision(17);
  double previous_time = -std::numeric_limits<double>::infinity();
  for (const auto &state : trajectory.states) {
    const Eigen::Quaterniond attitude = state.attitude.normalized();
    if (!std::isfinite(state.time) || state.time <= previous_time ||
      !state.position.allFinite() || !state.velocity.allFinite() ||
      !state.attitude.coeffs().allFinite() || state.attitude.norm() < 1.0e-9 ||
      !std::isfinite(state.thrust_acceleration) || state.thrust_acceleration <= 0.0 ||
      !state.body_rate.allFinite())
    {
      stream.close();
      std::remove(temporary_path.c_str());
      return fail("trajectory contains an invalid or non-monotonic state");
    }
    stream << state.time << ','
           << state.position.x() << ',' << state.position.y() << ',' << state.position.z() << ','
           << state.velocity.x() << ',' << state.velocity.y() << ',' << state.velocity.z() << ','
           << attitude.w() << ',' << attitude.x() << ',' << attitude.y() << ',' << attitude.z() << ','
           << state.thrust_acceleration << ','
           << state.body_rate.x() << ',' << state.body_rate.y() << ',' << state.body_rate.z() << '\n';
    previous_time = state.time;
  }
  stream.close();
  if (!stream) {
    std::remove(temporary_path.c_str());
    return fail("failed while writing temporary trajectory file");
  }
  if (std::rename(temporary_path.c_str(), file_path.c_str()) != 0) {
    std::remove(temporary_path.c_str());
    return fail("cannot atomically replace output file: " + file_path);
  }
  return true;
}

OmTrajectoryResult loadOmTrajectoryCsv(const std::string &file_path)
{
  OmTrajectoryResult result;
  result.status = "cannot open saved omtraj file: " + file_path;
  std::ifstream stream(file_path);
  if (!stream.is_open()) {
    return result;
  }

  std::string line;
  std::size_t line_number = 0U;
  double previous_time = -std::numeric_limits<double>::infinity();
  while (std::getline(stream, line)) {
    ++line_number;
    const std::size_t first_character = line.find_first_not_of(" \t\r");
    if (first_character == std::string::npos || line[first_character] == '#') {
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    OmTrajectoryState state;
    double qw = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    if (!(values >> state.time >> state.position.x() >> state.position.y() >> state.position.z()
      >> state.velocity.x() >> state.velocity.y() >> state.velocity.z()
      >> qw >> qx >> qy >> qz >> state.thrust_acceleration
      >> state.body_rate.x() >> state.body_rate.y() >> state.body_rate.z()))
    {
      result.states.clear();
      result.status = "invalid saved omtraj row at line " + std::to_string(line_number);
      return result;
    }
    double unexpected_value = 0.0;
    if (values >> unexpected_value) {
      result.states.clear();
      result.status = "too many columns in saved omtraj row at line " +
        std::to_string(line_number);
      return result;
    }
    state.attitude = Eigen::Quaterniond(qw, qx, qy, qz);
    if (!std::isfinite(state.time) || state.time <= previous_time ||
      !state.position.allFinite() || !state.velocity.allFinite() ||
      !state.attitude.coeffs().allFinite() || state.attitude.norm() < 1.0e-9 ||
      !std::isfinite(state.thrust_acceleration) || state.thrust_acceleration <= 0.0 ||
      !state.body_rate.allFinite())
    {
      result.states.clear();
      result.status = "non-finite/non-monotonic saved omtraj state at line " +
        std::to_string(line_number);
      return result;
    }
    state.attitude.normalize();
    if (!result.states.empty() &&
      result.states.back().attitude.coeffs().dot(state.attitude.coeffs()) < 0.0)
    {
      state.attitude.coeffs() *= -1.0;
    }
    result.states.push_back(state);
    previous_time = state.time;
  }
  if (!stream.eof() || result.states.size() < 2U ||
    std::abs(result.states.front().time) > 1.0e-9)
  {
    result.states.clear();
    result.status = "saved omtraj must contain at least two states and start at t=0";
    return result;
  }
  result.success = true;
  result.converged = true;
  result.total_time = result.states.back().time;
  result.status = "saved omtraj loaded";
  return result;
}

OmTrajectoryState OmTrajectoryOptimizer::sample(
  const OmTrajectoryResult &trajectory, double time)
{
  // (F15) 对包围时刻 t_i<=t<t_{i+1}，alpha=(t-t_i)/(t_{i+1}-t_i)。
  // p,v,a_T,omega 使用 (1-alpha)y_i+alpha*y_{i+1}；姿态使用最短路 SLERP。
  // 因此 MPC 可以按控制周期采样参考，但它不会增加离线配点约束的分辨率。
  OmTrajectoryState output;
  if (trajectory.states.empty()) {
    return output;
  }
  if (time <= trajectory.states.front().time) {
    return trajectory.states.front();
  }
  if (time >= trajectory.states.back().time) {
    return trajectory.states.back();
  }
  auto upper = std::upper_bound(
    trajectory.states.begin(), trajectory.states.end(), time,
    [](double value, const OmTrajectoryState &state) {return value < state.time;});
  const auto lower = upper - 1;
  const double duration = upper->time - lower->time;
  const double alpha = duration > 1.0e-9 ? (time - lower->time) / duration : 0.0;
  output.time = time;
  output.position = (1.0 - alpha) * lower->position + alpha * upper->position;
  output.velocity = (1.0 - alpha) * lower->velocity + alpha * upper->velocity;
  output.attitude = lower->attitude.slerp(alpha, upper->attitude).normalized();
  output.thrust_acceleration =
    (1.0 - alpha) * lower->thrust_acceleration + alpha * upper->thrust_acceleration;
  output.body_rate = (1.0 - alpha) * lower->body_rate + alpha * upper->body_rate;
  return output;
}
