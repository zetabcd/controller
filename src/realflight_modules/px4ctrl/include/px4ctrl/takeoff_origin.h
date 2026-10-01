#pragma once

#include <Eigen/Core>
#include <stdexcept>

namespace px4ctrl
{
// Translation only: measurements and controller references stay in world NWU.
class TakeoffOrigin
{
public:
  void update(bool armed, bool position_valid, const Eigen::Vector3d &position)
  {
    if (armed) {was_armed_ = true; return;}
    if (was_armed_) {takeoff_pending_ = true;}
    was_armed_ = false;
    valid_ = position_valid && position.allFinite();
    if (valid_) {origin_ = position;}
  }

  bool valid() const {return valid_;}
  bool takeoffPending() const {return takeoff_pending_;}
  const Eigen::Vector3d &position() const {return origin_;}

  Eigen::Vector3d enterHover(const Eigen::Vector3d &hold_position)
  {
    if (!valid_ || !hold_position.allFinite()) {
      throw std::invalid_argument("No valid ground takeoff origin / hover position");
    }
    const Eigen::Vector3d target = takeoff_pending_ ?
      Eigen::Vector3d(origin_ + Eigen::Vector3d(0.0, 0.0, 0.2)) : hold_position;
    takeoff_pending_ = false;
    return target;
  }

private:
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  bool valid_{false};
  bool takeoff_pending_{true};
  bool was_armed_{false};
};
}  // namespace px4ctrl
