#include <px4ctrl/acados_nmpc.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace
{
Eigen::Quaterniond attitudeFromForce(const Eigen::Vector3d & force, double yaw)
{
  const Eigen::Vector3d z = force.norm() > 1.0e-8 ? force.normalized() : Eigen::Vector3d::UnitZ();
  Eigen::Vector3d y = z.cross(Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0));
  if (y.norm() < 1.0e-8) {y = z.unitOrthogonal();}
  y.normalize();
  Eigen::Matrix3d r;
  r.col(0) = y.cross(z);r.col(1) = y;r.col(2) = z;
  return Eigen::Quaterniond(r);
}
}

void AcadosNmpcControl::configure(const AcadosNmpcOptions & options, double mass)
{
  if (!std::isfinite(mass) || mass <= 0) {
    throw std::invalid_argument("NMPC mass must be positive");
  }
  auto solver = std::make_unique<AcadosNmpcSolver>(options);
  references_.resize(options.horizon + 1);
  solver_ = std::move(solver);
  mass_ = mass;
}
const AcadosNmpcOptions & AcadosNmpcControl::options() const
{
  if (!solver_) {throw std::logic_error("Configure acados before use");}
  return solver_->options();
}
const AcadosNmpcDiagnostics & AcadosNmpcControl::diagnostics() const
{
  if (!solver_) {throw std::logic_error("Configure acados before use");}
  return solver_->diagnostics();
}
void AcadosNmpcControl::reset() {if (solver_) {solver_->reset();}}
void AcadosNmpcControl::prepareReferences(
  const px4ctrl::ReferenceWindow & window, const Eigen::Quaterniond & attitude)
{
  const auto & o = options();
  px4ctrl::validateReferenceWindow(window, o.horizon, o.prediction_dt);
  auto nominal = window;
  const bool compensate_drag = o.linear_drag.squaredNorm() > 0 || o.horizontal_lift > 0;
  if (compensate_drag) {
    for (auto & r : nominal.points) {
      const auto rotation = r.attitude.toRotationMatrix();
      const Eigen::Vector3d vb = rotation.transpose() * r.velocity;
      Eigen::Vector3d aero = -o.linear_drag.cwiseProduct(vb);
      aero.z() += o.horizontal_lift * vb.head<2>().squaredNorm();
      const Eigen::Vector3d force = rotation *
        (Eigen::Vector3d(0, 0, r.thrust_acceleration) - aero);
      r.attitude = attitudeFromForce(force, r.yaw);
      r.thrust_acceleration = force.norm();
    }
    // The old source has no aerodynamic derivatives. Reconstruct derivatives
    // of the adjusted attitude explicitly; do not reuse incompatible old FF.
    for (int k = 0; k < o.horizon; ++k) {
      const Eigen::AngleAxisd delta(nominal.points[k].attitude.conjugate() *
        nominal.points[k + 1].attitude);
      nominal.points[k].body_rate = delta.angle() * delta.axis() / o.prediction_dt;
    }
    nominal.points.back().body_rate = nominal.points[o.horizon - 1].body_rate;
    px4ctrl::differentiateAngularRate(nominal);
  }
  feedforward_valid_ = nominal.points.front().angular_acceleration_valid;
  auto differentiated = nominal;
  px4ctrl::differentiateAngularRate(differentiated);
  for (int k = 0; k <= o.horizon; ++k) {
    const auto & r = nominal.points[k];
    auto & out = references_[k];
    out.state = {r.position, r.velocity, r.attitude, r.body_rate};
    out.input = {r.thrust_acceleration, r.body_rate};
    // Each shooting interval holds body-axis feedforward components, just as
    // RatesThrustSetpoint does. Stage zero uses the measured body frame.
    out.angular_acceleration_ff = r.angular_acceleration_valid ?
      r.body_acceleration : Eigen::Vector3d::Zero();
    if (k == 0) {out.angular_acceleration_ff = px4ctrl::angularFeedforward(r, attitude);}
    const Eigen::Vector3d nominal_alpha = r.angular_acceleration_valid ?
      r.body_acceleration : differentiated.points[k].body_acceleration;
    // omega_dot=(u-omega)/tau+alpha_ff. Invert only the UNCOMPENSATED part.
    out.input.body_rate += o.rate_time_constant.cwiseProduct(
      nominal_alpha - out.angular_acceleration_ff);
  }
}

px4debug_msgs::msg::Px4ctrlDebug AcadosNmpcControl::calculate(
  const px4ctrl::ReferenceWindow & window,
  const Ref_State_t & ref, const AcadosNmpcState & current,
  double now, double elapsed, Control_Setpoint_t & out)
{
  const auto & o = options();
  out.rate_dot_ref.setZero();
  out.rate_dot_ref_valid = false;
  if (ref.fsm_state == 1) {
    reset();
    auto error = current.attitude.normalized().conjugate() * ref.q.normalized();
    if (error.w() < 0) {error.coeffs() *= -1.0;}
    out.bodyrates = 8.0 * error.vec();
    out.bodyrates.z() += ref.yaw_rate;
    if (!out.bodyrates.allFinite()) {out.bodyrates.setZero();}
    out.bodyrates = out.bodyrates.cwiseMax(-o.body_rate_max).cwiseMin(o.body_rate_max);
    const double throttle = std::isfinite(ref.throttle) ? std::clamp(ref.throttle, 0.0, 1.0) : 0.0;
    out.thrust = mass_ * (o.thrust_acceleration_min +
      throttle * (o.thrust_acceleration_max - o.thrust_acceleration_min));
    out.q = ref.q;
  } else {
    if (!std::isfinite(now) || std::abs(window.stamp - now) > 1e-6) {
      throw std::invalid_argument("Stale NMPC reference window");
    }
    prepareReferences(window, current.attitude);
    const auto u = solver_->step(current, references_, elapsed);
    out.thrust = mass_ * u.thrust_acceleration;
    out.bodyrates = u.body_rate;
    out.q = references_.front().state.attitude;
    if (solver_->diagnostics().solved) {
      out.rate_dot_ref = references_.front().angular_acceleration_ff;
      out.rate_dot_ref_valid = feedforward_valid_;
    }
  }
  if (!out.q.coeffs().allFinite() || out.q.norm() < 1.0e-8) {out.q.setIdentity();} else {
    out.q.normalize();
  }
  px4debug_msgs::msg::Px4ctrlDebug debug;
  const Eigen::Vector3d p = ref.fsm_state == 1 ? ref.p : references_.front().state.position;
  const Eigen::Vector3d v = ref.fsm_state == 1 ? ref.v : references_.front().state.velocity;
  debug.ref_p_x = p.x();debug.ref_p_y = -p.y();debug.ref_p_z = -p.z();
  debug.ref_v_x = v.x();debug.ref_v_y = -v.y();debug.ref_v_z = -v.z();
  debug.des_q_w = out.q.w();debug.des_q_x = out.q.x();
  debug.des_q_y = -out.q.y();debug.des_q_z = -out.q.z();
  debug.des_rate_x = out.bodyrates.x();debug.des_rate_y = -out.bodyrates.y();
  debug.des_rate_z = -out.bodyrates.z();debug.des_thrust = out.thrust;
  debug.ref_rate_dot_x = out.rate_dot_ref.x();
  debug.ref_rate_dot_y = -out.rate_dot_ref.y();
  debug.ref_rate_dot_z = -out.rate_dot_ref.z();
  debug.timestamp = std::isfinite(now) && now >= 0 ? static_cast<uint64_t>(now * 1.0e6) : 0;
  if (ref.fsm_state != 1) {
    const auto & d = diagnostics();
    debug.nmpc_active = true;
    debug.nmpc_solved = d.solved;
    debug.nmpc_fallback = d.fallback;
    debug.nmpc_status = d.status;
    debug.nmpc_iterations = d.iterations;
    debug.nmpc_consecutive_failures = d.consecutive_failures;
    debug.nmpc_solve_time_ms = d.solve_time_ms;
    debug.nmpc_objective = d.objective;
    debug.nmpc_residual = d.residual;
  }
  return debug;
}
