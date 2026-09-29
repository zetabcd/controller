#include <px4ctrl/legacy_trajectory_reference.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace px4ctrl
{
void LegacyTrajectoryReference::setTrajectory(OmTrajectoryResult trajectory, double start)
{
  if (!trajectory.success || trajectory.states.empty() || !std::isfinite(start)) {
    throw std::invalid_argument("Invalid legacy trajectory");
  }
  double previous = -std::numeric_limits<double>::infinity();
  for (const auto & s : trajectory.states) {
    if (!std::isfinite(s.time) || s.time <= previous || !s.position.allFinite() ||
      !s.velocity.allFinite() || !s.attitude.coeffs().allFinite() ||
      !std::isfinite(s.attitude.norm()) || s.attitude.norm() < 1e-8 ||
      !std::isfinite(s.thrust_acceleration) || s.thrust_acceleration <= 0 || !s.body_rate.allFinite())
    {
      throw std::invalid_argument("Invalid or unordered legacy trajectory sample");
    }
    previous = s.time;
  }
  trajectory_ = std::move(trajectory);
  start_ = start;
}
void LegacyTrajectoryReference::clearTrajectory() {trajectory_ = {};}

ReferenceWindow LegacyTrajectoryReference::sample(double now, int horizon, double dt, double gravity) const
{
  if (!active() || horizon < 0 || !std::isfinite(now) || !std::isfinite(dt) || dt <= 0) {
    throw std::invalid_argument("Invalid legacy sampling request");
  }
  ReferenceWindow result{now, dt, {}};
  for (int k = 0; k <= horizon; ++k) {
    const auto s = OmTrajectoryOptimizer::sample(trajectory_, now - start_ + k * dt);
    ReferencePoint r;
    r.full_state = true;
    r.position = s.position;r.velocity = s.velocity;r.attitude = s.attitude;
    r.thrust_acceleration = s.thrust_acceleration;r.body_rate = s.body_rate;
    // This source has no angular acceleration. Reconstruct it explicitly from
    // adjacent original samples, never by differentiating feedback commands.
    const double t = now - start_ + k * dt;
    const auto & states = trajectory_.states;
    if (states.size() > 1 && t >= states.front().time && t < states.back().time) {
      auto upper = std::upper_bound(states.begin(), states.end(), t,
        [](double time, const OmTrajectoryState & state) {return time < state.time;});
      const auto lower = upper - 1;
      const Eigen::Vector3d alpha_world =
        (upper->attitude.normalized() * upper->body_rate -
        lower->attitude.normalized() * lower->body_rate) / (upper->time - lower->time);
      r.body_acceleration = s.attitude.normalized().conjugate() * alpha_world;
      r.angular_acceleration_valid = true;
    }
    result.points.push_back(resolveReference(r, gravity));
  }
  return result;
}
}  // namespace px4ctrl
