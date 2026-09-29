#include <px4ctrl/acados_nmpc_solver.h>
#include "acados_solver_px4ctrl_nmpc.h"
#include <acados_c/ocp_nlp_interface.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace
{
using StateArray = std::array<double, 13>;
using InputArray = std::array<double, 4>;
using Clock = std::chrono::steady_clock;
static_assert(
  PX4CTRL_NMPC_NX == 13 && PX4CTRL_NMPC_NU == 4 &&
  PX4CTRL_NMPC_NP == 12 && PX4CTRL_NMPC_NY0 == 18 &&
  PX4CTRL_NMPC_NY == 14 && PX4CTRL_NMPC_NYN == 10,
  "Regenerate acados with script/generate_px4ctrl_acados_nmpc.py");

bool validState(const AcadosNmpcState & s)
{
  return s.position.allFinite() && s.velocity.allFinite() && s.body_rate.allFinite() &&
         s.attitude.coeffs().allFinite() && std::isfinite(s.attitude.norm()) &&
         s.attitude.norm() > 1.0e-8;
}
bool validInput(const AcadosNmpcInput & u)
{
  return std::isfinite(u.thrust_acceleration) && u.body_rate.allFinite();
}
StateArray pack(const AcadosNmpcState & s)
{
  const auto q = s.attitude.normalized();
  return {s.position.x(), s.position.y(), s.position.z(),
    s.velocity.x(), s.velocity.y(), s.velocity.z(), q.w(), q.x(), q.y(), q.z(),
    s.body_rate.x(), s.body_rate.y(), s.body_rate.z()};
}
InputArray pack(const AcadosNmpcInput & u)
{
  return {u.thrust_acceleration, u.body_rate.x(), u.body_rate.y(), u.body_rate.z()};
}
void validate(const AcadosNmpcOptions & o)
{
  const auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  if (o.horizon < 1 || o.horizon > 200 || !positive(o.prediction_dt) ||
    !positive(o.gravity) || !o.rate_time_constant.allFinite() ||
    (o.rate_time_constant.array() < o.prediction_dt).any() ||
    !o.linear_drag.allFinite() || (o.linear_drag.array() < 0).any() ||
    !std::isfinite(o.horizontal_lift) || o.horizontal_lift < 0 ||
    !std::isfinite(o.thrust_acceleration_min) ||
    o.thrust_acceleration_min < 0 || !positive(o.thrust_acceleration_max) ||
    o.thrust_acceleration_max <= o.thrust_acceleration_min ||
    !o.body_rate_max.allFinite() || (o.body_rate_max.array() <= 0).any() ||
    o.maximum_iterations < 1 || o.maximum_iterations > 1000 ||
    !positive(o.tolerance) || !positive(o.solve_time_budget_ms) ||
    !o.state_weight.allFinite() || (o.state_weight.array() < 0).any() ||
    !o.terminal_weight.allFinite() || (o.terminal_weight.array() < 0).any() ||
    !o.input_weight.allFinite() || (o.input_weight.array() <= 0).any() ||
    !o.command_change_weight.allFinite() || (o.command_change_weight.array() < 0).any() ||
    !o.fallback_position_gain.allFinite() || (o.fallback_position_gain.array() < 0).any() ||
    !o.fallback_velocity_gain.allFinite() || (o.fallback_velocity_gain.array() < 0).any() ||
    !positive(o.fallback_attitude_gain))
  {
    throw std::invalid_argument("Invalid acados NMPC numerical configuration");
  }
}
}  // namespace

class AcadosNmpcSolver::Impl
{
public:
  explicit Impl(const AcadosNmpcOptions & o)
  : options(o)
  {
    validate(o);
    capsule = px4ctrl_nmpc_acados_create_capsule();
    if (!capsule) {throw std::bad_alloc();}
    std::vector<double> steps(o.horizon, o.prediction_dt);
    // Generated code supports dynamic N for this homogeneous shooting problem.
    const int status = px4ctrl_nmpc_acados_create_with_discretization(
      capsule, o.horizon, steps.data());
    if (status != 0) {
      px4ctrl_nmpc_acados_free(capsule);
      px4ctrl_nmpc_acados_free_capsule(capsule);
      capsule = nullptr;
      throw std::runtime_error("acados creation failed, status=" + std::to_string(status));
    }
    configure();
    states.resize(o.horizon + 1);
    inputs.resize(o.horizon);
    reset();
  }
  ~Impl()
  {
    if (capsule) {
      px4ctrl_nmpc_acados_free(capsule);
      px4ctrl_nmpc_acados_free_capsule(capsule);
    }
  }
  void setX(int k, StateArray & x)
  {
    ocp_nlp_out_set(
      capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out,
      capsule->nlp_in, k, "x", x.data());
  }
  void setU(int k, InputArray & u)
  {
    ocp_nlp_out_set(
      capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out,
      capsule->nlp_in, k, "u", u.data());
  }
  void configure()
  {
    auto * c = capsule;
    int iterations = options.maximum_iterations;
    ocp_nlp_solver_opts_set(c->nlp_config, c->nlp_opts, "max_iter", &iterations);
    double tol = options.tolerance;
    for (const auto * field : {"tol_stat", "tol_eq", "tol_ineq", "tol_comp"}) {
      ocp_nlp_solver_opts_set(c->nlp_config, c->nlp_opts, field, &tol);
    }
    double budget = options.solve_time_budget_ms * 1.0e-3;
    ocp_nlp_solver_opts_set(c->nlp_config, c->nlp_opts, "timeout_max_time", &budget);
    InputArray lo{options.thrust_acceleration_min, -options.body_rate_max.x(),
      -options.body_rate_max.y(), -options.body_rate_max.z()};
    InputArray hi{options.thrust_acceleration_max, options.body_rate_max.x(),
      options.body_rate_max.y(), options.body_rate_max.z()};
    std::array<double, 12> params{options.gravity, options.gravity, 0, 0, 0,
      options.rate_time_constant.x(), options.rate_time_constant.y(),
      options.rate_time_constant.z(),
      options.linear_drag.x(), options.linear_drag.y(), options.linear_drag.z(),
      options.horizontal_lift};
    for (int k = 0; k <= options.horizon; ++k) {
      px4ctrl_nmpc_acados_update_params(c, k, params.data(), params.size());
      const int ny = k == options.horizon ? 10 : (k == 0 ? 18 : 14);
      std::vector<double> w(ny * ny, 0.0);
      for (int i = 0; i < 10; ++i) {
        w[i * ny + i] = 2.0 * (k == options.horizon ?
          options.terminal_weight[i] : options.state_weight[i]);
      }
      if (k < options.horizon) {
        for (int i = 0; i < 4; ++i) {w[(10 + i) * ny + 10 + i] = 2.0 * options.input_weight[i];}
        ocp_nlp_constraints_model_set(
          c->nlp_config, c->nlp_dims, c->nlp_in,
          c->nlp_out, k, "lbu", lo.data());
        ocp_nlp_constraints_model_set(
          c->nlp_config, c->nlp_dims, c->nlp_in,
          c->nlp_out, k, "ubu", hi.data());
      }
      if (k == 0) {
        for (int i = 0; i < 4; ++i) {
          w[(14 + i) * ny + 14 + i] = 2.0 * options.command_change_weight[i];
        }
      }
      ocp_nlp_cost_model_set(c->nlp_config, c->nlp_dims, c->nlp_in, k, "W", w.data());
      // create_with_discretization sets stage scaling to dt. Our weights use
      // a discrete sum, so explicitly restore unit scaling for every node.
      double scale = 1.0;
      ocp_nlp_cost_model_set(c->nlp_config, c->nlp_dims, c->nlp_in, k, "scaling", &scale);
    }
  }
  void clearIterates()
  {
    has_solution = false;
    // Never reset numerical values: that would restore generated dt/W/parameters.
    px4ctrl_nmpc_acados_reset(capsule, 1, 0, 0, 0);
  }
  void reset()
  {
    clearIterates();
    diagnostics = {};
    last_command = {};
    last_command.thrust_acceleration = options.gravity;
    clamp(last_command);
  }
  void clamp(AcadosNmpcInput & u) const
  {
    u.thrust_acceleration = std::clamp(
      u.thrust_acceleration,
      options.thrust_acceleration_min, options.thrust_acceleration_max);
    u.body_rate = u.body_rate.cwiseMax(-options.body_rate_max).cwiseMin(options.body_rate_max);
  }
  AcadosNmpcInput fallback(
    const AcadosNmpcState & state,
    const AcadosNmpcReferences & refs) const
  {
    AcadosNmpcInput u;
    u.thrust_acceleration = options.gravity;
    if (validState(state)) {
      const auto q = state.attitude.normalized();
      const Eigen::Matrix3d r = q.toRotationMatrix();
      Eigen::Vector3d force = -options.fallback_velocity_gain.cwiseProduct(state.velocity);
      double yaw = std::atan2(r(1, 0), r(0, 0));
      if (!refs.empty() && validState(refs.front().state)) {
        const auto & ref = refs.front();
        force = options.fallback_position_gain.cwiseProduct(ref.state.position - state.position) +
          options.fallback_velocity_gain.cwiseProduct(ref.state.velocity - state.velocity);
        if (validInput(ref.input)) {
          force += ref.state.attitude.normalized() *
            Eigen::Vector3d(0, 0, ref.input.thrust_acceleration) -
            options.gravity * Eigen::Vector3d::UnitZ();
        }
        const auto rr = ref.state.attitude.normalized().toRotationMatrix();
        yaw = std::atan2(rr(1, 0), rr(0, 0));
      }
      force += options.gravity * Eigen::Vector3d::UnitZ();
      // Bounded geometric position/attitude feedback; no stale high-rate hold.
      if (force.allFinite() && force.norm() > 1.0e-8) {
        const Eigen::Vector3d z = force.normalized();
        Eigen::Vector3d y = z.cross(Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0));
        if (y.norm() < 1.0e-6) {y = z.unitOrthogonal();}
        y.normalize();
        Eigen::Matrix3d desired;
        desired.col(0) = y.cross(z);desired.col(1) = y;desired.col(2) = z;
        auto error = q.conjugate() * Eigen::Quaterniond(desired);
        if (error.w() < 0) {error.coeffs() *= -1.0;}
        u.body_rate = 2.0 * options.fallback_attitude_gain * error.vec();
        u.thrust_acceleration = force.dot(r.col(2));
      }
    }
    clamp(u);
    return u;
  }
  void initialize(
    const AcadosNmpcReferences & refs, const AcadosNmpcState & state,
    double elapsed)
  {
    const bool warm = has_solution && elapsed >= 0.0 &&
      elapsed < options.horizon * options.prediction_dt;
    const double shift = warm ? elapsed / options.prediction_dt : 0.0;
    const auto current = pack(state);
    const double sign = warm &&
      Eigen::Map<const Eigen::Vector4d>(states[0].data() + 6).dot(
      Eigen::Map<const Eigen::Vector4d>(current.data() + 6)) < 0 ? -1.0 : 1.0;
    for (int k = 0; k <= options.horizon; ++k) {
      StateArray x;
      if (warm) {
        const double index = std::min(k + shift, static_cast<double>(options.horizon));
        const int a = static_cast<int>(index), b = std::min(a + 1, options.horizon);
        const double f = index - a;
        for (int i = 0; i < 13; ++i) {x[i] = (1 - f) * states[a][i] + f * states[b][i];}
        Eigen::Quaterniond qa(states[a][6], states[a][7], states[a][8], states[a][9]);
        Eigen::Quaterniond qb(states[b][6], states[b][7], states[b][8], states[b][9]);
        auto q = qa.normalized().slerp(f, qb.normalized());
        x[6] = sign * q.w();x[7] = sign * q.x();x[8] = sign * q.y();x[9] = sign * q.z();
      } else {x = pack(refs[k].state);}
      setX(k, x);
      if (k < options.horizon) {
        auto u = pack(refs[k].input);
        if (warm) {
          // Controls are piecewise constant; resample at actual elapsed time.
          const int index = std::min(static_cast<int>(k + shift), options.horizon - 1);
          u = inputs[index];
        }
        setU(k, u);
      }
    }
  }
  AcadosNmpcInput step(
    const AcadosNmpcState & state,
    const AcadosNmpcReferences & refs, double elapsed)
  {
    const auto start = Clock::now();
    const int failures = diagnostics.consecutive_failures;
    diagnostics = {};
    const auto finish = [&]() {
        diagnostics.solve_time_ms =
          std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return last_command;
      };
    const auto fail = [&](int status) {
        diagnostics.status = status;
        diagnostics.fallback = true;
        diagnostics.consecutive_failures = failures + 1;
        clearIterates();
        last_command = fallback(state, refs);
        return finish();
      };
    if (!validState(state) || !std::isfinite(elapsed) || elapsed < 0 ||
      refs.size() != static_cast<std::size_t>(options.horizon + 1) ||
      !std::all_of(
        refs.begin(), refs.end(), [](const auto & r) {
          return validState(r.state) && validInput(r.input);
        })) {return fail(-1);}

    if (has_solution && elapsed >= options.horizon * options.prediction_dt) {
      clearIterates();
    }
    initialize(refs, state, elapsed);
    auto * c = capsule;
    auto x0 = pack(state);
    ocp_nlp_constraints_model_set(
      c->nlp_config, c->nlp_dims, c->nlp_in, c->nlp_out, 0, "lbx",
      x0.data());
    ocp_nlp_constraints_model_set(
      c->nlp_config, c->nlp_dims, c->nlp_in, c->nlp_out, 0, "ubx",
      x0.data());
    setX(0, x0);
    std::array<double, 12> params{options.gravity, last_command.thrust_acceleration,
      last_command.body_rate.x(), last_command.body_rate.y(), last_command.body_rate.z(),
      options.rate_time_constant.x(), options.rate_time_constant.y(),
      options.rate_time_constant.z(),
      options.linear_drag.x(), options.linear_drag.y(), options.linear_drag.z(),
      options.horizontal_lift};
    px4ctrl_nmpc_acados_update_params(c, 0, params.data(), params.size());
    Eigen::Vector4d previous_q = Eigen::Map<Eigen::Vector4d>(x0.data() + 6);
    for (int k = 0; k <= options.horizon; ++k) {
      std::array<double, 18> ref{};
      auto x = pack(refs[k].state);
      Eigen::Map<Eigen::Vector4d> q(x.data() + 6);
      if (q.dot(previous_q) < 0) {q *= -1.0;}
      previous_q = q;
      std::copy_n(x.begin(), 10, ref.begin());
      const auto u = pack(refs[k].input);
      std::copy(u.begin(), u.end(), ref.begin() + 10);
      ocp_nlp_cost_model_set(c->nlp_config, c->nlp_dims, c->nlp_in, k, "yref", ref.data());
      // Cold initialization must share the exact quaternion branch of yref.
      if (!has_solution) {setX(k, x);}
    }
    setX(0, x0);
    const int status = px4ctrl_nmpc_acados_solve(c);
    ocp_nlp_get(c->nlp_solver, "sqp_iter", &diagnostics.iterations);
    ocp_nlp_get(c->nlp_solver, "cost_value", &diagnostics.objective);
    ocp_nlp_out_get(
      c->nlp_config, c->nlp_dims, c->nlp_out, 0, "kkt_norm_inf",
      &diagnostics.residual);
    if (status != 0) {return fail(status);}
    if (std::chrono::duration<double, std::milli>(Clock::now() - start).count() >
      options.solve_time_budget_ms) {return fail(-3);}
    for (int k = 0; k <= options.horizon; ++k) {
      ocp_nlp_out_get(c->nlp_config, c->nlp_dims, c->nlp_out, k, "x", states[k].data());
      const Eigen::Map<const Eigen::Matrix<double, 13, 1>> x(states[k].data());
      if (!x.allFinite() || x.segment<4>(6).norm() < 1.0e-8) {return fail(-2);}
      if (k < options.horizon) {
        ocp_nlp_out_get(c->nlp_config, c->nlp_dims, c->nlp_out, k, "u", inputs[k].data());
        if (!Eigen::Map<const Eigen::Vector4d>(inputs[k].data()).allFinite()) {return fail(-2);}
      }
    }
    last_command.thrust_acceleration = inputs[0][0];
    last_command.body_rate = Eigen::Vector3d(inputs[0][1], inputs[0][2], inputs[0][3]);
    clamp(last_command);
    diagnostics.solved = true;
    has_solution = true;
    return finish();
  }
  AcadosNmpcOptions options;
  AcadosNmpcDiagnostics diagnostics;
  AcadosNmpcInput last_command;
  px4ctrl_nmpc_solver_capsule * capsule{nullptr};
  bool has_solution{false};
  std::vector<StateArray> states;
  std::vector<InputArray> inputs;
};

AcadosNmpcSolver::AcadosNmpcSolver(const AcadosNmpcOptions & o)
: impl_(std::make_unique<Impl>(o)) {}
AcadosNmpcSolver::~AcadosNmpcSolver() = default;
AcadosNmpcInput AcadosNmpcSolver::step(
  const AcadosNmpcState & s,
  const AcadosNmpcReferences & r, double dt) {return impl_->step(s, r, dt);}
void AcadosNmpcSolver::reset() {impl_->reset();}
const AcadosNmpcOptions & AcadosNmpcSolver::options() const {return impl_->options;}
const AcadosNmpcDiagnostics & AcadosNmpcSolver::diagnostics() const {return impl_->diagnostics;}
