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
void AcadosNmpcControl::setTrajectory(OmTrajectoryResult trajectory, const rclcpp::Time & start)
{
  trajectory_ = std::move(trajectory);
  trajectory_start_ = start.seconds();
  trajectory_active_ = trajectory_.success && !trajectory_.states.empty();
  reset();
}
void AcadosNmpcControl::clearTrajectory()
{
  trajectory_active_ = false;
  trajectory_.states.clear();
  reset();
}
void AcadosNmpcControl::sampleReferences(const Ref_State_t & ref, double now)
{
  const auto & o = options();
  if (trajectory_active_) {
    for (int k = 0; k <= o.horizon; ++k) {
      const auto sample = OmTrajectoryOptimizer::sample(
        trajectory_, now - trajectory_start_ + k * o.prediction_dt);
      references_[k] = {{sample.position, sample.velocity, sample.attitude, sample.body_rate},
        {sample.thrust_acceleration, sample.body_rate}};
    }
  } else {
    const auto r = ref.q.normalized().toRotationMatrix();
    const double yaw = std::atan2(r(1, 0), r(0, 0));
    for (int k = 0; k <= o.horizon; ++k) {
      const double t = k * o.prediction_dt;
      auto & out = references_[k];
      // Constant jerk extrapolation keeps position, velocity and acceleration consistent.
      out.state.position = ref.p + t * ref.v + 0.5 * t * t * ref.a + (t * t * t / 6.0) * ref.j;
      out.state.velocity = ref.v + t * ref.a + 0.5 * t * t * ref.j;
      const Eigen::Vector3d force = ref.a + t * ref.j + o.gravity * Eigen::Vector3d::UnitZ();
      out.state.attitude = attitudeFromForce(force, yaw + t * ref.yaw_rate);
      out.input.thrust_acceleration = std::clamp(
        force.norm(),
        o.thrust_acceleration_min, o.thrust_acceleration_max);
    }
  }
  const bool compensate_drag = o.linear_drag.squaredNorm() > 0 || o.horizontal_lift > 0;
  if (compensate_drag) {
    for (auto & r : references_) {
      const auto rotation = r.state.attitude.normalized().toRotationMatrix();
      const Eigen::Vector3d vb = rotation.transpose() * r.state.velocity;
      Eigen::Vector3d aero = -o.linear_drag.cwiseProduct(vb);
      aero.z() += o.horizontal_lift * vb.head<2>().squaredNorm();
      const Eigen::Vector3d force = rotation *
        (Eigen::Vector3d(0, 0, r.input.thrust_acceleration) - aero);
      const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
      r.state.attitude = attitudeFromForce(force, yaw);
      r.input.thrust_acceleration = force.norm();
    }
  }
  if (!trajectory_active_ || compensate_drag) {
    for (int k = 0; k < o.horizon; ++k) {
      const Eigen::AngleAxisd delta(references_[k].state.attitude.conjugate() *
        references_[k + 1].state.attitude);
      references_[k].input.body_rate = delta.angle() * delta.axis() / o.prediction_dt;
    }
    references_.back().input.body_rate = references_[o.horizon - 1].input.body_rate;
  }
  for (auto & r : references_) {
    r.state.body_rate = r.input.body_rate;
  }
  // Invert the nominal rate response for a dynamically consistent command reference.
  for (int k = 0; k < o.horizon; ++k) {
    const int a = k == 0 ? 0 : k - 1;
    const int b = std::min(k + 1, o.horizon - 1);
    if (b > a) {
      const Eigen::Vector3d rate_dot = (references_[b].state.body_rate -
        references_[a].state.body_rate) / ((b - a) * o.prediction_dt);
      references_[k].input.body_rate += o.rate_time_constant.cwiseProduct(rate_dot);
    }
  }
}

px4debug_msgs::msg::Px4ctrlDebug AcadosNmpcControl::calculate(
  const Ref_State_t & ref, const LocalPose_Data_t & pose, const Attitude_Data_t & attitude,
  const Eigen::Vector3d & measured_body_rate,
  double now, double elapsed, Control_Setpoint_t & out)
{
  const auto & o = options();
  if (ref.fsm_state == 1) {
    reset();
    auto error = attitude.q.normalized().conjugate() * ref.q.normalized();
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
    sampleReferences(ref, now);
    const AcadosNmpcState current{pose.p, pose.v, attitude.q, measured_body_rate};
    const auto u = solver_->step(current, references_, elapsed);
    out.thrust = mass_ * u.thrust_acceleration;
    out.bodyrates = u.body_rate;
    out.q = references_.front().state.attitude;
  }
  if (!out.q.coeffs().allFinite() || out.q.norm() < 1.0e-8) {out.q.setIdentity();} else {
    out.q.normalize();
  }
  out.rate_dot_ref.setZero();
  px4debug_msgs::msg::Px4ctrlDebug debug;
  const Eigen::Vector3d p = ref.fsm_state == 1 ? ref.p : references_.front().state.position;
  const Eigen::Vector3d v = ref.fsm_state == 1 ? ref.v : references_.front().state.velocity;
  debug.ref_p_x = p.x();debug.ref_p_y = -p.y();debug.ref_p_z = -p.z();
  debug.ref_v_x = v.x();debug.ref_v_y = -v.y();debug.ref_v_z = -v.z();
  debug.des_q_w = out.q.w();debug.des_q_x = out.q.x();
  debug.des_q_y = -out.q.y();debug.des_q_z = -out.q.z();
  debug.des_rate_x = out.bodyrates.x();debug.des_rate_y = -out.bodyrates.y();
  debug.des_rate_z = -out.bodyrates.z();debug.des_thrust = out.thrust;
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
