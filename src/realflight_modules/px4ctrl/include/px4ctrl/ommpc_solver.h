#pragma once

#include <px4ctrl/control_reference.h>
#include <memory>

// Lu et al. (2023), error-state QP on R^3 x R^3 x SO(3), extended with
// measured angular velocity and estimated thrust for this project's actuators.
// World z up, body FLU; attitude maps body -> world. No ROS dependencies.
struct OmMpcState
{
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude {Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rate {Eigen::Vector3d::Zero()};
  double thrust_acceleration {9.805};
};

struct OmMpcOptions
{
  int horizon {16};
  double prediction_dt {0.03};
  double gravity {9.805};
  Eigen::Vector3d rate_time_constant {0.10, 0.083, 0.25};
  double thrust_time_constant {0.033};
  Eigen::Vector3d linear_drag {Eigen::Vector3d::Zero()}; // k_d/m [1/s]
  double horizontal_lift {0.0}; // k_h/m [1/m]
  double thrust_acceleration_min {0.5}, thrust_acceleration_max {39.22};
  Eigen::Vector3d body_rate_max {Eigen::Vector3d::Constant(14.0)};
  // [position, velocity, Log(Rd^T R)]; extra actuator-state weights below.
  Eigen::Matrix < double, 9, 1 > state_weight {
    (Eigen::Matrix < double, 9, 1 > () << 60, 60, 80, 6, 6, 8, 3, 3, 2).finished()
  };
  double rate_weight {0.05}, thrust_weight {0.01};
  Eigen::Vector4d input_weight {0.10, 0.12, 0.12, 0.12};
  Eigen::Vector4d command_change_weight {0.02, 0.08, 0.08, 0.08};
  double terminal_weight_scale {2.0};
  int maximum_iterations {200};
  double tolerance {1e-4}, solve_time_budget_ms {6.0};
};

struct OmMpcDiagnostics
{
  bool solved {false}, fallback {false};
  int status {0}, iterations {0}, consecutive_failures {0};
  double cycle_time_ms {0.0}, solve_time_ms {0.0}, objective {0.0}, residual {0.0};
};

struct OmMpcCommand
{
  Eigen::Vector4d input {9.805, 0, 0, 0}; // [thrust/mass, commanded body rates]
  Eigen::Vector3d angular_acceleration {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude {Eigen::Quaterniond::Identity()}; // diagnostic target
};

// The same nonlinear discrete model is used for the affine defect and Jacobians.
// Exact first-order actuator responses, midpoint translation, SO(3) exponential.
OmMpcState ommpcPredict(
  const OmMpcState & state, const Eigen::Vector4d & input,
  const Eigen::Vector3d & angular_acceleration_world, double dt, const OmMpcOptions & options);

class OmMpcSolver
{
public:
  explicit OmMpcSolver(const OmMpcOptions & options);
  ~OmMpcSolver();
  OmMpcSolver(const OmMpcSolver &) = delete;
  OmMpcSolver & operator = (const OmMpcSolver &) = delete;
  OmMpcCommand step(OmMpcState state, const px4ctrl::ReferenceWindow & window, double dt);
  void reset();
  const OmMpcOptions & options() const;
  const OmMpcDiagnostics & diagnostics() const;

private:
  struct Impl;
  std::unique_ptr < Impl > impl_;
};
