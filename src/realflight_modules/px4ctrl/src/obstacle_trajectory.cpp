#include <px4ctrl/obstacle_trajectory.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4ctrl
{
ObstacleTriggeredTrajectory::ObstacleTriggeredTrajectory(const ObstacleTrajectoryOptions &o)
: options_(o)
{
  for (double v : {o.gravity, o.takeoff_height, o.takeoff_duration, o.settle_duration,
      o.trigger_distance, o.move_distance, o.move_duration, o.pose_timeout}) {
    if (!std::isfinite(v)) {throw std::invalid_argument("Nonfinite obstacle trajectory parameter");}
  }
  if (o.gravity <= 0 || o.takeoff_height < 0 || o.takeoff_duration <= 0 ||
      o.settle_duration < 0 || o.trigger_distance <= 0 || o.move_distance == 0 ||
      o.move_duration <= 0 || o.pose_timeout <= 0) {
    throw std::invalid_argument("Invalid obstacle trajectory distance/timing");
  }
  reset(0.0);
}

void ObstacleTriggeredTrajectory::reset(double yaw)
{
  ReferencePoint start;start.yaw = yaw;
  const Eigen::Vector3d hover(0, 0, options_.takeoff_height);
  takeoff_ = std::make_shared<PointToPointTrajectory>(start, hover,
    options_.takeoff_duration, options_.gravity);
  start.position = hover;
  move_ = std::make_shared<PointToPointTrajectory>(start,
    hover + options_.move_distance * Eigen::Vector3d::UnitX(),
    options_.move_duration, options_.gravity);
  move_start_ = last_update_ = -1.0;
}

bool ObstacleTriggeredTrajectory::update(double elapsed, const Eigen::Vector3d &aircraft,
  const Eigen::Vector3d &obstacle, double observation_age)
{
  if (!std::isfinite(elapsed) || elapsed < 0 || elapsed < last_update_ - 1e-6) {
    throw std::invalid_argument("Invalid/backwards obstacle trajectory clock");
  }
  last_update_ = elapsed;
  if (triggered() || elapsed < readyTime() || !aircraft.allFinite() ||
      !obstacle.allFinite() || !std::isfinite(observation_age) || observation_age < 0 ||
      observation_age > options_.pose_timeout ||
      (obstacle-aircraft).norm() >= options_.trigger_distance) {return false;}
  move_start_ = elapsed;
  return true;
}

double ObstacleTriggeredTrajectory::duration() const
{return triggered() ? move_start_ + options_.move_duration : readyTime();}

std::vector<double> ObstacleTriggeredTrajectory::boundaries() const
{
  std::vector<double> result{0.0, options_.takeoff_duration, readyTime()};
  if (triggered()) {result.push_back(move_start_);result.push_back(duration());}
  return result;
}

ReferencePoint ObstacleTriggeredTrajectory::evaluate(double time) const
{
  if (!std::isfinite(time)) {throw std::invalid_argument("Nonfinite obstacle trajectory time");}
  if (triggered() && time >= move_start_) {return move_->evaluate(time-move_start_);}
  return takeoff_->evaluate(time);
}

bool ObstaclePosition::accept(const Eigen::Vector3d &position, double stamp, double now,
  const std::string &frame, double timeout)
{
  if (!position.allFinite() || !std::isfinite(stamp) || !std::isfinite(now) ||
      !std::isfinite(timeout) || timeout <= 0 || stamp <= 0 || stamp <= stamp_ ||
      now < stamp || now-stamp > timeout ||
      ((!frame_.empty() || frame_locked_) && frame != frame_)) {return false;}
  if (!frame_locked_) {frame_ = frame;frame_locked_ = true;}
  position_ = position;stamp_ = stamp;received_ = now;
  return true;
}

double ObstaclePosition::age(double now) const
{
  if (!std::isfinite(now) || stamp_ < 0 || received_ < 0 || now < received_) {
    return std::numeric_limits<double>::infinity();
  }
  return now-stamp_;
}
}  // namespace px4ctrl
