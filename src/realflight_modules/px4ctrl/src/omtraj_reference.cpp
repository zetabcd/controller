#include <px4ctrl/sampled_trajectory.h>
#include <px4ctrl/omtraj.h>
#include <stdexcept>

namespace px4ctrl
{
std::shared_ptr<const Trajectory> loadOmTrajectoryReference(
  const std::string & path,
  double gravity)
{
  const auto result = loadOmTrajectoryCsv(path);
  if (!result.success || result.states.size() < 2) {throw std::invalid_argument(result.status);}
  const auto & first = result.states.front();
  const auto R = first.attitude.toRotationMatrix();
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const Eigen::Quaterniond inverse(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
  TimedReferences points;points.reserve(result.states.size());
  for (const auto & s:result.states) {
    ReferencePoint r;r.full_state = true;
    r.position = inverse * (s.position - first.position);r.velocity = inverse * s.velocity;
    r.attitude = (inverse * s.attitude).normalized();r.thrust_acceleration = s.thrust_acceleration;
    r.body_rate = s.body_rate;
    points.push_back({s.time - first.time, r});
  }
  return std::make_shared<SampledTrajectory>(std::move(points), gravity);
}
}
