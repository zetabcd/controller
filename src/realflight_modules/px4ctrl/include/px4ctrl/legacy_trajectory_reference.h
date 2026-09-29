#pragma once

#include <px4ctrl/control_reference.h>
#include <px4ctrl/omtraj.h>

namespace px4ctrl
{
// Temporary boundary for existing trajectories. Controllers never own or read
// OmTrajectoryResult. Replace this source when the planners are revised.
class LegacyTrajectoryReference
{
public:
  void setTrajectory(OmTrajectoryResult trajectory, double start);
  void clearTrajectory();
  bool active() const {return !trajectory_.states.empty();}
  ReferenceWindow sample(double now, int horizon, double dt, double gravity) const;

private:
  OmTrajectoryResult trajectory_;
  double start_{0.0};
};
}  // namespace px4ctrl
