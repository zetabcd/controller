#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace px4ctrl
{
// Mode metadata is separate from motion references. No duplicate p/v/a path.
struct ControlModeReference
{
  int fsm_state{0};
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  double yaw_rate{0.0}, throttle{0.0};
  bool position_valid{true}, velocity_valid{true}, acceleration_valid{true};
};

// Local z-up world, FLU body. q maps reference-body vectors into world.
// The adapter must use the SAME world convention as state feedback (the current
// PX4 bridge uses NED -> NWU). No ROS or planner types belong in this contract.
struct ReferencePoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  double yaw{0.0}, yaw_rate{0.0}, yaw_acceleration{0.0};
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  double thrust_acceleration{0.0};  // total nominal thrust / mass [m/s^2]
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_acceleration{Eigen::Vector3d::Zero()};
  // false: p/v/a/j/s/yaw derivatives are authoritative; resolve analytically.
  // true: p/v/q/a_T/omega are authoritative; no invented jerk or snap.
  bool full_state{false};
  bool angular_acceleration_valid{false};
  // True only when p/v/a/jerk/snap come from one differentiable curve.
  bool kinematics_valid{false};
};
using ReferencePoints = std::vector<ReferencePoint, Eigen::aligned_allocator<ReferencePoint>>;

// Per-call snapshot; point k is valid at stamp + k*dt. Not a persistent preview.
struct ReferenceWindow
{
  double stamp{0.0};
  double dt{0.0};
  ReferencePoints points;
};

ReferencePoint resolveReference(ReferencePoint point, double gravity);
// Geometric auxiliary axis, valid through inversion. Unlike Euler yaw this
// constrains a projected body axis; singular projections are rejected.
enum class HeadingAxis {BodyX, BodyY};
ReferencePoint resolveGeometricReference(
  ReferencePoint point, double gravity, const Eigen::Vector3d & heading,
  HeadingAxis axis = HeadingAxis::BodyX);
// One nominal-model aerodynamic correction. Analytic sources retain analytic
// attitude derivatives; sampled sources require explicit reconstruction after it.
ReferencePoint compensateReferenceAerodynamics(
  ReferencePoint point, double gravity, const Eigen::Vector3d & drag, double lift);
ReferenceWindow extrapolateFlatReference(
  const ReferencePoint & point, double stamp, int horizon, double dt, double gravity);
void validateReferenceWindow(const ReferenceWindow & window, int horizon, double dt);
// Convert a physical angular acceleration, not the derivative of a transported
// rate command. Inner-loop feedback owns the tracking-error contribution.
Eigen::Vector3d angularFeedforward(
  const ReferencePoint & reference, const Eigen::Quaterniond & actual_attitude);
// Explicit reconstruction for sampled sources / model-adjusted attitudes.
// Differentiates world angular velocity then expresses it in reference body.
void differentiateAngularRate(ReferenceWindow & window);
}  // namespace px4ctrl
