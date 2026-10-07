#pragma once

#include <px4ctrl/trajectory.h>
#include <utility>

namespace px4ctrl
{
struct ObstacleTrajectoryOptions
{
  double gravity{9.805};
  double takeoff_height{0.5}, takeoff_duration{3.0}, settle_duration{0.5};
  double trigger_distance{1.0}, move_distance{1.0}, move_duration{3.0};
  double pose_timeout{0.2};
};

// Coordinates are translated world NWU, NOT rotated by the entry heading.
// update() is called once per control tick, never from prediction sampling.
class ObstacleTriggeredTrajectory final : public Trajectory
{
public:
  explicit ObstacleTriggeredTrajectory(const ObstacleTrajectoryOptions &options);
  void reset(double yaw);
  bool update(double elapsed, const Eigen::Vector3d &aircraft,
    const Eigen::Vector3d &obstacle, double observation_age);
  bool triggered() const {return move_start_ >= 0.0;}
  double readyTime() const {return options_.takeoff_duration + options_.settle_duration;}
  // Before triggering, this is the time the stationary waiting endpoint begins.
  double duration() const override;
  std::vector<double> boundaries() const override;
  ReferencePoint evaluate(double time) const override;
private:
  ObstacleTrajectoryOptions options_;
  std::shared_ptr<PointToPointTrajectory> takeoff_, move_;
  double move_start_{-1.0}, last_update_{-1.0};
};

// Only position is used. The source must already share the controller world
// coordinates; a matching frame label is not a geometric calibration.
class ObstaclePosition
{
public:
  explicit ObstaclePosition(std::string frame = "") : frame_(std::move(frame)) {}
  bool accept(const Eigen::Vector3d &position, double stamp, double now,
    const std::string &frame, double timeout);
  double age(double now) const;
  void clear() {stamp_ = received_ = -1.0;}
  const Eigen::Vector3d &position() const {return position_;}
private:
  std::string frame_;
  bool frame_locked_{false};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  double stamp_{-1.0}, received_{-1.0};
};
}  // namespace px4ctrl
