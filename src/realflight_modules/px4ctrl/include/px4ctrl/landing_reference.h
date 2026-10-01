#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4ctrl
{
// A landing request is explicit: a generic CMD -> HOVER fallback never creates it.
class LandingReference
{
public:
  enum class Phase {Idle, Requested, Settling, Descending, Landed};
  void request() {phase_ = Phase::Requested;}
  void cancel() {phase_ = Phase::Idle;}
  bool active() const {return phase_ != Phase::Idle;}
  bool requested() const {return phase_ == Phase::Requested;}
  bool landed() const {return phase_ == Phase::Landed;}
  Phase phase() const {return phase_;}

  void start(const Eigen::Vector3d &position, double ground_z)
  {
    if (!position.allFinite() || !std::isfinite(ground_z)) {
      throw std::invalid_argument("Invalid landing origin");
    }
    position_ = position;
    ground_z_ = ground_z;
    velocity_.setZero();
    acceleration_.setZero();
    stable_time_ = touchdown_time_ = 0.0;
    phase_ = Phase::Settling;
  }

  void update(double dt, const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
    bool feedback_valid, bool upright_and_slow_rotation, bool confirmed_landed)
  {
    acceleration_.setZero();
    if (!active() || requested() || landed()) {return;}
    dt = std::isfinite(dt) ? std::clamp(dt, 0.0, 0.05) : 0.0;
    if (!feedback_valid || !position.allFinite() || !velocity.allFinite()) {
      velocity_.setZero();
      stable_time_ = touchdown_time_ = 0.0;
      return;
    }
    const bool stable = upright_and_slow_rotation && velocity.norm() < 0.2;
    if (phase_ == Phase::Settling) {
      stable_time_ = stable ? stable_time_ + dt : 0.0;
      if (stable_time_ < 0.5) {return;}
      phase_ = Phase::Descending;
    }

    // Height alone must never stop motors: PX4 must also report landed freshly.
    const bool touchdown = confirmed_landed && stable &&
      std::abs(position.z() - ground_z_) < 0.15;
    touchdown_time_ = touchdown ? touchdown_time_ + dt : 0.0;
    if (touchdown_time_ >= 0.5) {
      phase_ = Phase::Landed;
      position_.z() = std::min(position_.z(), std::max(ground_z_ - 0.1, position.z() - 0.05));
      velocity_.setZero();
      return;
    }

    // Bound the descent reference; reserve 10 cm below takeoff height for contact.
    const double floor = ground_z_ - 0.1;
    const double remaining = std::max(0.0, position_.z() - floor);
    const double height = std::min(position.z(), position_.z()) - ground_z_;
    const double speed = std::clamp(0.1 + 0.4 * (height - 0.3), 0.1, 0.3);
    const double desired_vz = -std::min(speed, std::sqrt(2.0 * 0.3 * remaining));
    const double old_vz = velocity_.z();
    velocity_.z() += std::clamp(desired_vz - old_vz, -0.3 * dt, 0.3 * dt);
    // Never move upwards if the estimate is already below the nominal floor.
    const double next_z = position_.z() + velocity_.z() * dt;
    if (next_z <= floor) {
      position_.z() = std::min(position_.z(), floor);
      velocity_.z() = 0.0;
    } else {
      position_.z() = next_z;
    }
    if (dt > 0.0) {acceleration_.z() = (velocity_.z() - old_vz) / dt;}
  }

  const Eigen::Vector3d &position() const {return position_;}
  const Eigen::Vector3d &velocity() const {return velocity_;}
  const Eigen::Vector3d &acceleration() const {return acceleration_;}

private:
  Phase phase_{Phase::Idle};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
  double ground_z_{0.0}, stable_time_{0.0}, touchdown_time_{0.0};
};
}  // namespace px4ctrl
