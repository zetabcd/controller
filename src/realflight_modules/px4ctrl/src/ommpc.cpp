#include <px4ctrl/ommpc.h>
#include <px4ctrl/px4ctrl_node.h>
#include <algorithm>

OmMpcControl::OmMpcControl(PX4ControlNode & node, const OmMpcOptions & options)
: node_(node), solver_(std::make_unique<OmMpcSolver>(options)) {}
void OmMpcControl::setOptions(const OmMpcOptions & options)
{solver_ = std::make_unique<OmMpcSolver>(options);}
void OmMpcControl::resetControlParams() {solver_->reset();}
void OmMpcControl::resetThrustMapping(const Parameter_t &) {}
bool OmMpcControl::estimateThrustModel(const Eigen::Vector3d &) {return false;}

px4debug_msgs::msg::Px4ctrlDebug OmMpcControl::calculateControl(
  const px4ctrl::ReferenceWindow & window, const px4ctrl::ControlModeReference & reference,
  const LocalPose_Data_t & pose, const Attitude_Data_t & attitude,
  const Sensor_Data_t & sensor, const double & dt,
  Control_Setpoint_t & out, const Parameter_t & parameters)
{
  out.rate_dot_ref.setZero();out.rate_dot_ref_valid = false;
  px4debug_msgs::msg::Px4ctrlDebug debug;
  if (reference.fsm_state == 1) {
    solver_->reset();
    Eigen::Quaterniond error = attitude.q.normalized().conjugate() *
      reference.attitude.normalized();
    if (error.w() < 0) {error.coeffs() *= -1;}
    out.bodyrates = 8.0 * error.vec();out.bodyrates.z() += reference.yaw_rate;
    if (!out.bodyrates.allFinite()) {out.bodyrates.setZero();}
    out.bodyrates = out.bodyrates.cwiseMax(-options().body_rate_max).cwiseMin(
      options().body_rate_max);
    const double throttle = std::isfinite(reference.throttle) ? std::clamp(
      reference.throttle, 0.,
      1.) : 0.;
    out.thrust = 4 *
      (parameters.motor.u_min + throttle * (parameters.motor.u_max - parameters.motor.u_min));
    out.q = reference.attitude;
  } else {
    const auto command = solver_->step(
      {pose.p, pose.v, attitude.q, sensor.w,
        options().gravity}, window, dt);
    out.thrust = parameters.uav.mass * command.input(0);
    out.bodyrates = command.input.tail<3>();out.q = command.attitude;
    out.rate_dot_ref = command.angular_acceleration;
    const auto & d = solver_->diagnostics();
    out.rate_dot_ref_valid = d.solved;
    const auto & r = window.points.front();
    debug.ref_p_x = r.position.x();debug.ref_p_y = -r.position.y();debug.ref_p_z = -r.position.z();
    debug.ref_v_x = r.velocity.x();debug.ref_v_y = -r.velocity.y();debug.ref_v_z = -r.velocity.z();
    debug.ref_a_x = r.acceleration.x();debug.ref_a_y = -r.acceleration.y();
    debug.ref_a_z = -r.acceleration.z();
    debug.ommpc_active = true;debug.ommpc_solved = d.solved;debug.ommpc_fallback = d.fallback;
    debug.ommpc_status = d.status;debug.ommpc_iterations = d.iterations;
    debug.ommpc_consecutive_failures = d.consecutive_failures;
    debug.ommpc_cycle_time_ms = d.cycle_time_ms;debug.ommpc_solve_time_ms = d.solve_time_ms;
    debug.ommpc_objective = d.objective;debug.ommpc_residual = d.residual;
  }
  debug.des_q_w = out.q.w();debug.des_q_x = out.q.x();debug.des_q_y = -out.q.y();
  debug.des_q_z = -out.q.z();
  debug.des_rate_x = out.bodyrates.x();debug.des_rate_y = -out.bodyrates.y();
  debug.des_rate_z = -out.bodyrates.z();
  debug.des_thrust = out.thrust;
  debug.ref_rate_dot_x = out.rate_dot_ref.x();debug.ref_rate_dot_y = -out.rate_dot_ref.y();
  debug.ref_rate_dot_z = -out.rate_dot_ref.z();
  debug.timestamp = node_.get_clock()->now().nanoseconds() / 1000;
  return debug;
}
