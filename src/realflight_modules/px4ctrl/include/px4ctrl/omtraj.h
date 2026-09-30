#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <string>
#include <vector>

// Manuscript III: x=(p,v,R), right SO(3) increments, u=(a_T,omega).
// Local z-up world / FLU body; R maps body to world (PX4 bridge: NWU).
struct OmTrajectoryState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double time {0};
  Eigen::Vector3d position {Eigen::Vector3d::Zero()}, velocity {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude {Eigen::Quaterniond::Identity()};
  double thrust_acceleration {9.805};
  Eigen::Vector3d body_rate {Eigen::Vector3d::Zero()};
};
using OmStates = std::vector<OmTrajectoryState, Eigen::aligned_allocator<OmTrajectoryState>>;
struct OmTrajectoryWaypoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position {Eigen::Vector3d::Zero()};
  double tolerance {0.12};
  double maximum_tilt {3.14159265358979323846}; // waypoint-only body-z cone
};
using OmWaypoints = std::vector<OmTrajectoryWaypoint,
    Eigen::aligned_allocator<OmTrajectoryWaypoint>>;
struct OmTrajectoryBoundary
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position {Eigen::Vector3d::Zero()}, velocity {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude {Eigen::Quaterniond::Identity()};
};
struct OmTrajectoryModel
{
  double gravity {9.805};
  Eigen::Vector3d linear_drag {Eigen::Vector3d::Zero()}; // manuscript II-B: kd/m [1/s]
  double horizontal_lift {0}; // kh/m [1/m]
};
// Physical constraints, never objective terms. Bounds include feedback reserve.
struct OmTrackingLimits
{
  bool enabled {true};
  double thrust_min {1.0}, thrust_max {30.0};
  Eigen::Vector3d rate_max {Eigen::Vector3d::Constant(4.0)};
  double thrust_slew_max {40.0};
  double thrust_time_constant {0.033};
  Eigen::Vector3d angular_acceleration_max {Eigen::Vector3d::Constant(16.0)};
  // Angular acceleration feedforward is available: nominal command is omega.
  // rate_max reserves feedback authority; alpha/motor bounds limit maneuvering.
  double speed_max {6.0}, maximum_tilt {1.5707963267948966}, minimum_relative_altitude {0.0};
  bool motor_constraints {true};
  double mass {0.811}, arm {0.1216}, arm_angle {0.7853981633974483};
  Eigen::Vector3d inertia {1.91e-3, 2.37e-3, 3.60e-3};
  double torque_to_thrust {0.011405}, motor_min {0.5}, motor_max {8.0}; // N
};
struct OmTrajectoryIteration
{
  int scp_iteration {0};
  double total_time {0}, maximum_update {0}, maximum_dynamics_defect {0};
  double maximum_virtual_control {0}, maximum_waypoint_residual {0};
  double maximum_order_residual {0};
  double step_solve_time {0}, qp_solve_time {0}, elapsed_solve_time {0};
  std::string solver_status;
  OmStates states;
};
struct OmTrajectoryOptions
{
  int intervals {40}, max_scp_iterations {80}, integration_substeps {12};
  double minimum_time {0.5}, maximum_time {30.0}, initial_speed {2.5};
  bool optimize_total_time {true}, enforce_hover_boundary_input {true};
  OmTrajectoryModel model;
  OmTrackingLimits tracking;
  double thrust_acceleration_min {0.5}, thrust_acceleration_max {39.22};
  Eigen::Vector3d body_rate_max {Eigen::Vector3d::Constant(14.0)};
  // Numerical parameters, separate from physical constraints.
  double position_trust_region {0.75}, velocity_trust_region {2.0};
  double attitude_trust_region {0.35}, thrust_trust_region {5.0};
  Eigen::Vector3d body_rate_trust_region {Eigen::Vector3d::Constant(2.0)};
  double time_trust_region {0.15}, progress_trust_region {0.35};
  double state_regularization {0.02}, progress_regularization {0.1};
  double virtual_control_weight {10.0}, cstc_merit_weight {100.0};
  double maximum_virtual_control_weight {1000.0};
  double convergence_tolerance {1e-3};
  double position_tolerance {2e-5}, velocity_tolerance {2e-5}, attitude_tolerance {2e-5};
  double constraint_tolerance {2e-5}, cstc_tolerance {1e-6};
  double cstc_relaxation_initial {0.005}, cstc_relaxation_decay {0.6};
  int scp_backtracking_steps {8};
  bool store_history {false};
  std::function<void(const OmTrajectoryIteration &)> progress_callback;
};
struct OmTrajectoryValidation
{
  bool valid {false};
  std::string reason;
  double position_defect {0}, velocity_defect {0}, attitude_defect {0};
  double constraint_violation {0}, waypoint_violation {0}, integration_error {0};
  std::vector<double> waypoint_times;
  std::vector<std::pair<int, double>> refinement_points;
};
struct OmTrajectoryResult
{
  bool success {false}, converged {false}, dynamics_validated {false};
  // v2 records the model and FOH input. v1 is inspection-only.
  int format_version {2}, integration_substeps {8}, scp_iterations {0};
  OmTrajectoryModel model;
  std::string status;
  double total_time {0}, total_solve_time {0}, maximum_update {0};
  double maximum_dynamics_defect {0}, maximum_virtual_control {0};
  double maximum_waypoint_residual {0}, maximum_order_residual {0};
  std::vector<double> waypoint_times;
  Eigen::MatrixXd progress_lambda, progress_mu;
  OmStates states;
  std::vector<OmTrajectoryIteration> history;
};
class OmTrajectoryOptimizer
{
public:
  explicit OmTrajectoryOptimizer(const OmTrajectoryOptions & options = {});
  void setOptions(const OmTrajectoryOptions & options);
  const OmTrajectoryOptions & options() const {return options_;}
  OmTrajectoryResult optimize(
    const OmTrajectoryBoundary & initial,
    const OmTrajectoryBoundary & terminal, const OmWaypoints & waypoints);
  // Integrates the same continuous model/FOH input; stationary endpoint holds.
  static OmTrajectoryState sample(const OmTrajectoryResult & trajectory, double time);

private:
  OmTrajectoryOptions options_;
};
OmTrajectoryValidation validateOmTrajectory(
  const OmTrajectoryResult &, const OmTrajectoryOptions &, const OmWaypoints & waypoints = {});
bool saveOmTrajectoryCsv(
  const OmTrajectoryResult &, const std::string &,
  std::string * error = nullptr);
OmTrajectoryResult loadOmTrajectoryCsv(const std::string &);
