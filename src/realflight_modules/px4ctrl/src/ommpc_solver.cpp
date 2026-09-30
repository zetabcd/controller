#include <px4ctrl/ommpc_solver.h>
#include <osqp/osqp.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace
{
using Error = Eigen::Matrix<double, 13, 1>;
using A = Eigen::Matrix<double, 13, 13>;
using B = Eigen::Matrix<double, 13, 4>;
using Clock = std::chrono::steady_clock;
Eigen::Quaterniond exp(const Eigen::Vector3d & v)
{
  const double a = v.norm();
  if (a < 1e-10) {return Eigen::Quaterniond(1, v.x() / 2, v.y() / 2, v.z() / 2).normalized();}
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, v / a));
}
Eigen::Vector3d log(Eigen::Quaterniond q)
{
  q.normalize();
  if (q.w() < 0) {q.coeffs() *= -1;}
  const Eigen::AngleAxisd aa(q);
  return aa.angle() * aa.axis();
}
bool valid(const OmMpcState & s)
{
  return s.position.allFinite() && s.velocity.allFinite() && s.body_rate.allFinite() &&
         s.attitude.coeffs().allFinite() && std::isfinite(s.attitude.norm()) &&
         s.attitude.norm() > 1e-8 &&
         std::isfinite(s.thrust_acceleration);
}
OmMpcState retract(OmMpcState s, const Error & e)
{
  s.position += e.head<3>();s.velocity += e.segment<3>(3);
  s.attitude = s.attitude * exp(e.segment<3>(6));
  s.body_rate += e.segment<3>(9);s.thrust_acceleration += e(12);
  return s;
}
Error difference(const OmMpcState & s, const OmMpcState & r)
{
  Error e;
  e << s.position - r.position, s.velocity - r.velocity,
    log(r.attitude.conjugate() * s.attitude), s.body_rate - r.body_rate,
    s.thrust_acceleration - r.thrust_acceleration;
  return e;
}
Eigen::Vector3d acceleration(const OmMpcState & s, const OmMpcOptions & o)
{
  const Eigen::Vector3d vb = s.attitude.conjugate() * s.velocity;
  Eigen::Vector3d ab = -o.linear_drag.cwiseProduct(vb);
  ab.z() += s.thrust_acceleration + o.horizontal_lift * vb.head<2>().squaredNorm();
  return s.attitude * ab - o.gravity * Eigen::Vector3d::UnitZ();
}
void validate(const OmMpcOptions & o)
{
  if (o.horizon < 2 || o.horizon > 60 || !std::isfinite(o.prediction_dt) ||
    o.prediction_dt < .001 || o.prediction_dt > .1 || !std::isfinite(o.gravity) || o.gravity <= 0 ||
    !o.rate_time_constant.allFinite() || (o.rate_time_constant.array() <= 0).any() ||
    !std::isfinite(o.thrust_time_constant) || o.thrust_time_constant <= 0 ||
    !o.linear_drag.allFinite() || (o.linear_drag.array() < 0).any() ||
    !std::isfinite(o.horizontal_lift) || o.horizontal_lift < 0 ||
    !std::isfinite(o.thrust_acceleration_min) || o.thrust_acceleration_min < 0 ||
    !std::isfinite(o.thrust_acceleration_max) ||
    o.thrust_acceleration_max <= o.thrust_acceleration_min ||
    !o.body_rate_max.allFinite() || (o.body_rate_max.array() <= 0).any() ||
    !o.state_weight.allFinite() || (o.state_weight.array() < 0).any() ||
    !o.input_weight.allFinite() || (o.input_weight.array() <= 0).any() ||
    !o.command_change_weight.allFinite() || (o.command_change_weight.array() < 0).any() ||
    !std::isfinite(o.rate_weight) || o.rate_weight < 0 ||
    !std::isfinite(o.thrust_weight) || o.thrust_weight < 0 ||
    !std::isfinite(o.terminal_weight_scale) || o.terminal_weight_scale <= 0 ||
    o.maximum_iterations < 1 || !std::isfinite(o.tolerance) || o.tolerance <= 0 ||
    !std::isfinite(o.solve_time_budget_ms) || o.solve_time_budget_ms <= 0)
  {throw std::invalid_argument("Invalid OMMPC model, weights, limits or solver settings");}
}
}

OmMpcState ommpcPredict(
  const OmMpcState & s, const Eigen::Vector4d & u,
  const Eigen::Vector3d & alpha_world, double h, const OmMpcOptions & o)
{
  OmMpcState mid = s, out = s;
  const Eigen::Array3d eh = (-.5 * h / o.rate_time_constant.array()).exp();
  const Eigen::Vector3d target = u.tail<3>() +
    o.rate_time_constant.cwiseProduct(s.attitude.conjugate() * alpha_world);
  mid.body_rate = (eh * s.body_rate.array() + (1 - eh) * target.array()).matrix();
  mid.attitude = (s.attitude * exp(.5 * h * s.body_rate)).normalized();
  mid.thrust_acceleration = u(0) + (s.thrust_acceleration - u(0)) * std::exp(
    -.5 * h / o.thrust_time_constant);
  mid.velocity = s.velocity + .5 * h * acceleration(s, o);
  const Eigen::Vector3d acc = acceleration(mid, o);
  out.position = s.position + h * s.velocity + .5 * h * h * acc;
  out.velocity = s.velocity + h * acc;
  out.attitude = (s.attitude * exp(h * mid.body_rate)).normalized();
  const Eigen::Vector3d target_mid = u.tail<3>() +
    o.rate_time_constant.cwiseProduct(mid.attitude.conjugate() * alpha_world);
  out.body_rate =
    (eh.square() * s.body_rate.array() + (1 - eh.square()) * target_mid.array()).matrix();
  out.thrust_acceleration = u(0) + (s.thrust_acceleration - u(0)) * std::exp(
    -h / o.thrust_time_constant);
  return out;
}

struct OmMpcSolver::Impl
{
  OmMpcOptions o;
  OmMpcDiagnostics d;
  OSQPSolver * solver{nullptr};
  // Fixed dense upper triangle and identity sparsity; reuse the OSQP workspace.
  std::vector<OSQPFloat> pv, av, q, lower, upper;
  std::vector<OSQPInt> pi, pp, ai, ap;
  Eigen::VectorXd previous;
  Eigen::Vector4d last;
  double thrust;
  bool history{false};
  explicit Impl(const OmMpcOptions & options)
  : o(options)
  {
    validate(o);
    const int n = 4 * o.horizon;
    q.resize(n);lower.resize(n);upper.resize(n);
    pp.push_back(0);ap.push_back(0);
    for (int j = 0; j < n; ++j) {
      for (int i = 0; i <= j; ++i) {pi.push_back(i);pv.push_back(i == j ? 1 : 0);}
      pp.push_back(pv.size());ai.push_back(j);av.push_back(1);ap.push_back(j + 1);
      lower[j] = -OSQP_INFTY;upper[j] = OSQP_INFTY;
    }
    OSQPCscMatrix pmat, amat;
    OSQPCscMatrix_set_data(&pmat, n, n, pv.size(), pv.data(), pi.data(), pp.data());
    OSQPCscMatrix_set_data(&amat, n, n, n, av.data(), ai.data(), ap.data());
    OSQPSettings settings;osqp_set_default_settings(&settings);
    settings.verbose = 0;settings.polishing = 0;settings.warm_starting = 1;
    settings.max_iter = o.maximum_iterations;
    settings.eps_abs = o.tolerance;settings.eps_rel = o.tolerance;
    settings.time_limit = o.solve_time_budget_ms / 1000.;
    // Identity setup otherwise fixes a poor scale for subsequent online Hessians.
    settings.scaling = 0;
    if (osqp_setup(
        &solver, &pmat, q.data(), &amat, lower.data(), upper.data(), n, n,
        &settings) != 0)
    {
      if (solver) {osqp_cleanup(solver);solver = nullptr;}
      throw std::runtime_error("OMMPC OSQP setup failed");
    }
    reset();
  }
  ~Impl() {if (solver) {osqp_cleanup(solver);}}
  void reset()
  {
    d = {};history = false;thrust = o.gravity;last << o.gravity, 0, 0, 0;
    previous = Eigen::VectorXd::Zero(4 * o.horizon);
    osqp_cold_start(solver);
  }
  Eigen::Vector4d clamp(Eigen::Vector4d u) const
  {
    u(0) = std::clamp(u(0), o.thrust_acceleration_min, o.thrust_acceleration_max);
    u.tail<3>() = u.tail<3>().cwiseMax(-o.body_rate_max).cwiseMin(o.body_rate_max);
    return u;
  }
  OmMpcCommand fallback(const OmMpcState & s, const px4ctrl::ReferencePoint & r)
  {
    d.fallback = true;++d.consecutive_failures;history = false;osqp_cold_start(solver);
    OmMpcCommand cmd;
    cmd.input << o.gravity, 0, 0, 0;
    if (valid(s)) {
      // Current-state feedback; never keep a stale high-rate flip command.
      Eigen::Vector3d force = o.gravity * Eigen::Vector3d::UnitZ() - 3.0 * s.velocity;
      if (r.position.allFinite() && r.velocity.allFinite()) {
        force += 2.5 * (r.position - s.position) + 3.0 * r.velocity;
      }
      if (force.norm() < 1e-6) {force = o.gravity * Eigen::Vector3d::UnitZ();}
      cmd.attitude =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), force.normalized());
      cmd.input << std::max(0., force.dot(s.attitude * Eigen::Vector3d::UnitZ())),
        4.0 * log(s.attitude.conjugate() * cmd.attitude);
    }
    cmd.input = clamp(cmd.input);last = cmd.input;
    return cmd;
  }
  OmMpcCommand step(OmMpcState s, const px4ctrl::ReferenceWindow & window, double dt)
  {
    const auto started = Clock::now();
    const int failures = d.consecutive_failures;d = {};d.consecutive_failures = failures;
    const auto finish = [&](OmMpcCommand cmd) {
        d.cycle_time_ms = 1e3 * std::chrono::duration<double>(Clock::now() - started).count();
        return cmd;
      };
    // Reference contract violations belong to the FSM's invalid-reference recovery.
    px4ctrl::validateReferenceWindow(window, o.horizon, o.prediction_dt);
    // Wall-scheduled ROS loops may see the same /clock sample twice. No state
    // propagation or warm-start shift is needed then; zero is not a failure.
    if (!valid(s) || !std::isfinite(dt) || dt < 0 || dt > .2) {
      d.status = -1;return finish(fallback(s, window.points.front()));
    }
    s.attitude.normalize();
    thrust = last(0) + (thrust - last(0)) * std::exp(-dt / o.thrust_time_constant);
    s.thrust_acceleration = thrust;
    auto nominal = window;
    for (const auto & r : nominal.points) {
      if (r.aerodynamics_included &&
        ((r.model_linear_drag-o.linear_drag).norm()>1e-6 ||
        std::abs(r.model_horizontal_lift-o.horizontal_lift)>1e-6)) {
        throw std::invalid_argument("Trajectory/OMMPC aerodynamic model mismatch");
      }
    }
    if (o.linear_drag.squaredNorm() > 0 || o.horizontal_lift > 0) {
      for (auto & r:nominal.points) {
        r =
          px4ctrl::compensateReferenceAerodynamics(r, o.gravity, o.linear_drag, o.horizontal_lift);
      }
      // Sampled CSV has no analytic attitude derivatives after aero correction.
      if (std::any_of(
          nominal.points.begin(), nominal.points.end(),
          [](const auto & r) {return !r.angular_acceleration_valid;}))
      {
        for (int k = 0; k < o.horizon; ++k) {
          nominal.points[k].body_rate = log(
            nominal.points[k].attitude.conjugate() *
            nominal.points[k + 1].attitude) / o.prediction_dt;
        }
        nominal.points.back().body_rate = nominal.points[o.horizon - 1].body_rate;
        px4ctrl::differentiateAngularRate(nominal);
      }
    }
    // Missing CSV acceleration is reconstructed explicitly, also sent to the inner loop.
    if (std::any_of(
        nominal.points.begin(), nominal.points.end(),
        [](const auto & r) {return !r.angular_acceleration_valid;}))
    {
      px4ctrl::differentiateAngularRate(nominal);
    }
    const int n = 4 * o.horizon;
    std::vector<OmMpcState> refs(o.horizon + 1);
    Eigen::VectorXd ud(n);
    for (int k = 0; k <= o.horizon; ++k) {
      const auto & r = nominal.points[k];
      refs[k] = {r.position, r.velocity, r.attitude, r.body_rate, r.thrust_acceleration};
      if (k == o.horizon) {break;}
      const int a = std::max(0, k - 1), b = k + 1;
      const double thrust_dot = r.thrust_rate_valid ? r.thrust_rate :
        (nominal.points[b].thrust_acceleration -
        nominal.points[a].thrust_acceleration) / ((b - a) * o.prediction_dt);
      ud.segment<4>(4 * k) << r.thrust_acceleration + o.thrust_time_constant * thrust_dot,
        r.body_rate;
    }
    Error weights;
    weights << o.state_weight, Eigen::Vector3d::Constant(o.rate_weight), o.thrust_weight;
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n, n), gamma = Eigen::MatrixXd::Zero(13, n);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n);
    Error free = difference(s, refs[0]);
    // E_{k+1}=A E_k+B du_k+c. c retains reference/discretization defects.
    for (int k = 0; k < o.horizon; ++k) {
      const auto & r = nominal.points[k];
      const Eigen::Vector3d ff = r.attitude * r.body_acceleration;
      const Eigen::Vector4d u = ud.segment<4>(4 * k);
      const auto propagate = [&](const Error & dx, const Eigen::Vector4d & du) {
          return difference(
            ommpcPredict(
              retract(
                refs[k],
                dx), u + du, ff, o.prediction_dt, o), refs[k + 1]);
        };
      const Error c = propagate(Error::Zero(), Eigen::Vector4d::Zero());
      A a;B b;
      constexpr double eps = 1e-5;
      for (int i = 0; i < 13; ++i) {
        Error e = Error::Zero();e(i) = eps;
        a.col(i) = (propagate(e, Eigen::Vector4d::Zero()) -
          propagate(-e, Eigen::Vector4d::Zero())) / (2 * eps);
      }
      for (int i = 0; i < 4; ++i) {
        Eigen::Vector4d e = Eigen::Vector4d::Zero();e(i) = eps;
        b.col(i) = (propagate(Error::Zero(), e) - propagate(Error::Zero(), -e)) / (2 * eps);
      }
      free = (a * free + c).eval();gamma = (a * gamma).eval();gamma.block<13, 4>(0, 4 * k) += b;
      const Error wk = weights * ((k == o.horizon - 1) ? o.terminal_weight_scale : 1.);
      h.noalias() += gamma.transpose() * wk.asDiagonal() * gamma;
      gradient.noalias() += gamma.transpose() * wk.asDiagonal() * free;
      h.block<4, 4>(4 * k, 4 * k).diagonal() += o.input_weight;
    }
    if (history) {
      h.topLeftCorner<4, 4>().diagonal() += o.command_change_weight;
      gradient.head<4>() += o.command_change_weight.cwiseProduct(ud.head<4>() - last);
    }
    if (!h.allFinite() || !gradient.allFinite()) {
      d.status = -2;return finish(fallback(s, window.points.front()));
    }
    // Scale the whole objective by one scalar; does not change its minimizer.
    const double scale = std::max(1., h.diagonal().maxCoeff());
    int nz = 0;
    for (int j = 0; j < n; ++j) {
      for (int i = 0; i <= j; ++i) {pv[nz++] = 2 * h(i, j) / scale;}
      q[j] = 2 * gradient(j) / scale;
      const int axis = j % 4;
      lower[j] = (axis == 0 ? o.thrust_acceleration_min : -o.body_rate_max(axis - 1)) - ud(j);
      upper[j] = (axis == 0 ? o.thrust_acceleration_max : o.body_rate_max(axis - 1)) - ud(j);
    }
    // Warm-start absolute previous commands on the new reference/time grid.
    // Fractional shift matters: control dt is usually smaller than prediction_dt.
    Eigen::VectorXd warm = Eigen::VectorXd::Zero(n), dual = Eigen::VectorXd::Zero(n);
    if (history) {
      for (int k = 0; k < o.horizon; ++k) {
        const double at = std::min(double(o.horizon - 1), k + dt / o.prediction_dt);
        const int i = static_cast<int>(at), j = std::min(i + 1, o.horizon - 1);
        warm.segment<4>(4 * k) = (1 - (at - i)) * previous.segment<4>(4 * i) +
          (at - i) * previous.segment<4>(4 * j) - ud.segment<4>(4 * k);
      }
    }
    int status = osqp_update_data_mat(solver, pv.data(), nullptr, pv.size(), nullptr, nullptr, 0);
    status |= osqp_update_data_vec(solver, q.data(), lower.data(), upper.data());
    if (status) {d.status = -2;return finish(fallback(s, window.points.front()));}
    osqp_warm_start(solver, warm.data(), dual.data());
    const double used = 1e3 * std::chrono::duration<double>(Clock::now() - started).count();
    if (used >= o.solve_time_budget_ms) {
      d.status = -3;return finish(fallback(s, window.points.front()));
    }
    OSQPSettings settings = *solver->settings;
    settings.time_limit = (o.solve_time_budget_ms - used) / 1000.;
    osqp_update_settings(solver, &settings);
    if (!status) {status = osqp_solve(solver);}
    d.status = solver->info->status_val;d.iterations = solver->info->iter;
    d.solve_time_ms = 1e3 * solver->info->solve_time;
    d.objective = solver->info->obj_val * scale;
    d.residual = std::max(solver->info->prim_res, solver->info->dual_res);
    d.solved = status == 0 && (d.status == OSQP_SOLVED || d.status == OSQP_SOLVED_INACCURATE) &&
      solver->solution && solver->solution->x;
    Eigen::VectorXd solution;
    if (d.solved) {solution = Eigen::Map<Eigen::VectorXd>(solver->solution->x, n);}
    if (d.solved && !solution.allFinite()) {d.status = -2;d.solved = false;}
    if (!d.solved) {
      d.solved = false;return finish(fallback(s, window.points.front()));
    }
    previous = solution + ud;
    OmMpcCommand cmd;cmd.input = clamp(previous.head<4>());
    cmd.attitude = nominal.points.front().attitude;
    cmd.angular_acceleration = px4ctrl::angularFeedforward(nominal.points.front(), s.attitude);
    last = cmd.input;history = true;d.consecutive_failures = 0;
    return finish(cmd);
  }
};

OmMpcSolver::OmMpcSolver(const OmMpcOptions & o)
: impl_(std::make_unique<Impl>(o)) {}
OmMpcSolver::~OmMpcSolver() = default;
void OmMpcSolver::reset() {impl_->reset();}
const OmMpcOptions & OmMpcSolver::options() const {return impl_->o;}
const OmMpcDiagnostics & OmMpcSolver::diagnostics() const {return impl_->d;}
OmMpcCommand OmMpcSolver::step(OmMpcState s, const px4ctrl::ReferenceWindow & w, double dt)
{return impl_->step(s, w, dt);}
