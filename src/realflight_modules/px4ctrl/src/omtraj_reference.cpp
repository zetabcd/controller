#include <px4ctrl/sampled_trajectory.h>
#include <px4ctrl/omtraj_dynamics.h>
#include <algorithm>
#include <stdexcept>
namespace px4ctrl
{
namespace
{
class OmReference final : public Trajectory
{
  OmTrajectoryResult trajectory_;
  Eigen::Vector3d origin_;
  Eigen::Quaterniond inverse_;

public:
  explicit OmReference(OmTrajectoryResult r)
  : trajectory_(std::move(r))
  {
    const auto & first = trajectory_.states.front();
    origin_ = first.position;
    const auto R = first.attitude.toRotationMatrix();
    inverse_ =
      Eigen::Quaterniond(
      Eigen::AngleAxisd(
        -std::atan2(R(1, 0), R(0, 0)),
        Eigen::Vector3d::UnitZ()));
  }
  double duration() const override {return trajectory_.total_time;}
  std::vector<double> boundaries() const override
  {
    std::vector<double> t;
    for (const auto & x:trajectory_.states) {t.push_back(x.time);}
    return t;
  }
  ReferencePoint evaluate(double t) const override
  {
    const auto x = OmTrajectoryOptimizer::sample(trajectory_, t);
    ReferencePoint r;
    r.full_state = true;
    r.position = inverse_ * (x.position - origin_);
    r.velocity = inverse_ * x.velocity;
    r.attitude = (inverse_ * x.attitude).normalized();
    r.thrust_acceleration = x.thrust_acceleration;
    r.body_rate = x.body_rate;
    r.angular_acceleration_valid = true;
    r.thrust_rate_valid = true;
    r.aerodynamics_included = true;
    r.model_linear_drag = trajectory_.model.linear_drag;
    r.model_horizontal_lift = trajectory_.model.horizontal_lift;
    if (t >= 0 && t < duration()) {
      const auto & xs = trajectory_.states;
      auto b = std::upper_bound(
        xs.begin(), xs.end(), t, [](double s, const OmTrajectoryState & a) {
          return s < a.time;
        });
      const auto & a = *(b - 1);
      r.body_acceleration = (b->body_rate - a.body_rate) / (b->time - a.time);
      r.thrust_rate = (b->thrust_acceleration - a.thrust_acceleration) / (b->time - a.time);
    }
    return resolveReference(r, trajectory_.model.gravity);
  }
};
}
std::shared_ptr<const Trajectory> loadOmTrajectoryReference(
  const std::string & path,
  double gravity)
{
  auto r = loadOmTrajectoryCsv(path);
  if (!r.success || !r.dynamics_validated || r.format_version != 2) {
    throw std::invalid_argument(
            "Omtraj requires validated v2 CSV; regenerate legacy file: " + r.status);
  }
  if (std::abs(gravity - r.model.gravity) > 1e-6) {
    throw std::invalid_argument("Omtraj/controller gravity mismatch");
  }
  for (const auto * x:{&r.states.front(), &r.states.back()}) {
    if (x->velocity.norm() > 1e-4 || x->body_rate.norm() > 1e-4 ||
      std::abs(x->thrust_acceleration - gravity) > 1e-4 ||
      (x->attitude * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm() > 1e-4)
    {
      throw std::invalid_argument("Omtraj player requires stationary hover endpoints");
    }
  }
  return std::make_shared<OmReference>(std::move(r));
}
}
