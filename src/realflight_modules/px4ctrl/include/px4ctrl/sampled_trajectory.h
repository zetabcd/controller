#pragma once
#include <px4ctrl/trajectory.h>

namespace px4ctrl
{
struct TimedReference {double time; ReferencePoint point;};
using TimedReferences = std::vector<TimedReference, Eigen::aligned_allocator<TimedReference>>;
// File/sampled sources alone interpolate. Analytic sources never use this path.
class SampledTrajectory final : public Trajectory
{
public:
  SampledTrajectory(TimedReferences points, double gravity);
  ReferencePoint evaluate(double time) const override;
  double duration() const override {return points_.back().time;}

private:
  TimedReferences points_;
  ReferencePoint terminal_;
  double gravity_;
};
// Persistence boundary only. The optimizer and its CSV format remain unchanged.
std::shared_ptr<const Trajectory> loadOmTrajectoryReference(
  const std::string & path,
  double gravity);
}
