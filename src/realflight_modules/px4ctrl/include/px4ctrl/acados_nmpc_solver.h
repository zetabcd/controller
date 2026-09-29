#ifndef PX4CTRL_ACADOS_NMPC_SOLVER_H_
#define PX4CTRL_ACADOS_NMPC_SOLVER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

// ENU world, FLU body; q=[w,x,y,z], a_T=total thrust/mass [m/s^2].
struct AcadosNmpcState
{
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude {Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rate {Eigen::Vector3d::Zero()};  // measured FLU gyro [rad/s]
};

struct AcadosNmpcInput
{
  double thrust_acceleration {9.805};
  Eigen::Vector3d body_rate {Eigen::Vector3d::Zero()};
};

struct AcadosNmpcReference
{
  AcadosNmpcState state;
  AcadosNmpcInput input;
  // Known additive acceleration applied by the rate loop, FLU [rad/s^2].
  // Held constant over this shooting interval; not an optimization variable.
  Eigen::Vector3d angular_acceleration_ff{Eigen::Vector3d::Zero()};
};
using AcadosNmpcReferences = std::vector < AcadosNmpcReference,
  Eigen::aligned_allocator < AcadosNmpcReference >>;

struct AcadosNmpcOptions
{
  // All numerical options are runtime settings; no code generation needed.
  int horizon {12};
  double prediction_dt {0.04};
  double gravity {9.805};
  Eigen::Vector3d rate_time_constant {0.10, 0.083, 0.25};  // closed-loop response [s]
  Eigen::Vector3d linear_drag {Eigen::Vector3d::Zero()};  // body drag / mass [1/s]
  double horizontal_lift {0.0};  // kh / mass [1/m], zero-wind model
  double thrust_acceleration_min {0.5};
  double thrust_acceleration_max {39.22};
  Eigen::Vector3d body_rate_max {Eigen::Vector3d::Constant(14.0)};
  Eigen::Matrix < double, 10, 1 > state_weight {
    (Eigen::Matrix < double, 10, 1 > () << 18, 18, 18, 3, 3, 3, 5, 5, 5, 5).finished()
  };
  Eigen::Matrix < double, 10, 1 > terminal_weight {
    (Eigen::Matrix < double, 10, 1 > () << 35, 35, 35, 6, 6, 6, 10, 10, 10, 10).finished()
  };
  Eigen::Vector4d input_weight {0.10, 0.12, 0.12, 0.12};
  // Only u0 - last applied u, not differences between prediction stages.
  Eigen::Vector4d command_change_weight {0.02, 0.12, 0.12, 0.12};
  int maximum_iterations {60};
  double tolerance {1.0e-5};
  // SQP checks between iterations; not a hard preemptive real-time deadline.
  double solve_time_budget_ms {8.0};
  Eigen::Vector3d fallback_position_gain {Eigen::Vector3d::Constant(2.5)};
  Eigen::Vector3d fallback_velocity_gain {Eigen::Vector3d::Constant(3.0)};
  double fallback_attitude_gain {4.0};
};

struct AcadosNmpcDiagnostics
{
  bool solved {false};
  bool fallback {false};
  int status {0};  // acados status; -1 invalid input, -2 invalid solution, -3 wall budget.
  int iterations {0};
  int consecutive_failures {0};
  double solve_time_ms {0.0};  // entire step, including preparation and fallback
  double objective {0.0};
  double residual {0.0};
};

// Direct owner of the generated solver. No ROS, controller factories or backends.
// One instance per control loop, called synchronously from that loop.
class AcadosNmpcSolver
{
public:
  explicit AcadosNmpcSolver(const AcadosNmpcOptions & options);
  ~AcadosNmpcSolver();
  AcadosNmpcSolver(const AcadosNmpcSolver &) = delete;
  AcadosNmpcSolver & operator = (const AcadosNmpcSolver &) = delete;

  AcadosNmpcInput step(
    const AcadosNmpcState & state,
    const AcadosNmpcReferences & references, double elapsed_seconds);
  void reset();  // clears iterates/history, preserves all numerical configuration
  const AcadosNmpcOptions & options() const;
  const AcadosNmpcDiagnostics & diagnostics() const;

private:
  class Impl;
  std::unique_ptr < Impl > impl_;
};

#endif
