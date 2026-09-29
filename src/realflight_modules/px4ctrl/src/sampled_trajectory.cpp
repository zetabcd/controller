#include <px4ctrl/sampled_trajectory.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4ctrl
{
SampledTrajectory::SampledTrajectory(TimedReferences points, double gravity)
: points_(std::move(points)), gravity_(gravity)
{
  if (points_.size() < 2 || points_.front().time != 0) {
    throw std::invalid_argument("Sampled trajectory requires >=2 points starting at zero");
  }
  double previous = -1;
  for (auto & s:points_) {
    if (!std::isfinite(s.time) || s.time <= previous) {
      throw std::invalid_argument("Unordered sample times");
    }
    s.point = resolveReference(s.point, gravity_);previous = s.time;
  }
  // Explicitly reconstruct derivatives only for files that do not provide them.
  for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
    auto & a = points_[i];const auto & b = points_[i + 1];
    if (!a.point.angular_acceleration_valid) {
      a.point.body_acceleration = a.point.attitude.conjugate() *
        ((b.point.attitude * b.point.body_rate - a.point.attitude * a.point.body_rate) /
        (b.time - a.time));
      a.point.angular_acceleration_valid = true;
    }
  }
  terminal_.position = points_.back().point.position;
  terminal_.yaw = points_.back().point.yaw;
  terminal_ = resolveReference(terminal_, gravity_);
}
ReferencePoint SampledTrajectory::evaluate(double time) const
{
  if (!std::isfinite(time)) {throw std::invalid_argument("Nonfinite sample time");}
  if (time >= duration()) {return terminal_;}
  if (time <= 0) {return points_.front().point;}
  const auto upper = std::upper_bound(
    points_.begin(), points_.end(), time,
    [](double t, const TimedReference & s) {return t < s.time;});
  const auto & b = *upper, & a = *(upper - 1);
  const double u = (time - a.time) / (b.time - a.time);
  ReferencePoint r;
  r.full_state = true;r.position = (1 - u) * a.point.position + u * b.point.position;
  r.velocity = (1 - u) * a.point.velocity + u * b.point.velocity;
  r.attitude = a.point.attitude.slerp(u, b.point.attitude).normalized();
  r.thrust_acceleration = (1 - u) * a.point.thrust_acceleration + u * b.point.thrust_acceleration;
  r.body_rate = (1 - u) * a.point.body_rate + u * b.point.body_rate;
  const Eigen::Vector3d alpha_world = (b.point.attitude * b.point.body_rate -
    a.point.attitude * a.point.body_rate) / (b.time - a.time);
  r.body_acceleration = r.attitude.conjugate() * alpha_world;
  r.angular_acceleration_valid = true;
  return resolveReference(r, gravity_);
}
}
