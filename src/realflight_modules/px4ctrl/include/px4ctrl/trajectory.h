#pragma once

#include <px4ctrl/control_reference.h>
#include <memory>
#include <string>
#include <limits>

namespace px4ctrl
{
// All sources expose the same continuous-time contract; no planner result type
// crosses this boundary. Endpoints are explicit stationary holds.
class Trajectory
{
public:
  virtual ~Trajectory() = default;
  virtual ReferencePoint evaluate(double time) const = 0;
  virtual double duration() const = 0;
  virtual std::vector<double> boundaries() const {return {0.0, duration()};}
};

class TrajectoryPlayer
{
public:
  void start(
    std::shared_ptr<const Trajectory> source, double stamp,
    const Eigen::Vector3d & origin, double yaw);
  void clear();
  bool active() const {return static_cast<bool>(source_);}
  ReferenceWindow sample(double now, int horizon, double dt);

private:
  std::shared_ptr<const Trajectory> source_;
  double start_{0.0}, last_stamp_{0.0};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation_{Eigen::Quaterniond::Identity()};
};

// A C4 connector in world coordinates, with a stationary endpoint and fixed yaw.
// Uses the same polynomial construction as the analytic trajectories' takeoff.
class PointToPointTrajectory final : public Trajectory
{
public:
  PointToPointTrajectory(const ReferencePoint &start, const Eigen::Vector3d &target,
    double duration, double gravity);
  ReferencePoint evaluate(double time) const override;
  double duration() const override {return duration_;}
private:
  Eigen::Matrix<double, 3, 10> coefficients_;
  Eigen::Vector3d target_;
  double duration_, gravity_, yaw_;
};

enum class AnalyticPath {HorizontalCircle, VerticalCircle, Helix, FigureEight};
struct AnalyticTrajectoryOptions
{
  AnalyticPath path{AnalyticPath::FigureEight};
  double gravity{9.805};
  double takeoff_height{1.5}, takeoff_duration{3.0}, settle_duration{0.5};
  double radius{1.0};  // hard cap 1 m: main circle diameter <= 2 m
  int turns{2};  // horizontal circle: cruise turns, excludes ramps
  double speed{1.5}, ramp_duration{3.0};  // horizontal circle / eight
  double centripetal_g{1.8};  // vertical circle / helix: A/g > 1
  double pitch{0.5};  // axial advance PER REVOLUTION, m
  double entry_duration{0.85}, exit_duration{0.85};
  double entry_distance{1.43}, exit_distance{1.43};
  double connector_height{1.1}; // hover endpoints above the circle bottom
  double axis_transition_duration{1.0}; // helix: separate axial speed ramp
  double eight_length{2.0}, eight_width{1.2};
};

class AnalyticTrajectory final : public Trajectory
{
public:
  explicit AnalyticTrajectory(const AnalyticTrajectoryOptions & options);
  ReferencePoint evaluate(double time) const override;
  double duration() const override {return boundaries_.back();}
  std::vector<double> boundaries() const override {return boundaries_;}
  double mainStart() const {return main_start_;}
  double mainEnd() const {return main_end_;}

private:
  using Polynomial = Eigen::Matrix<double, 3, 10>;
  ReferencePoint main(double time) const;
  AnalyticTrajectoryOptions options_;
  Polynomial takeoff_, entry_, exit_, axis_entry_, axis_exit_;
  double entry_start_{0}, exit_end_{0};
  double main_start_{0}, main_end_{0}, cruise_duration_{0}, phase_rate_{0};
  Eigen::Vector3d main_origin_{Eigen::Vector3d::Zero()}, end_{Eigen::Vector3d::Zero()};
  std::vector<double> boundaries_;
  using QuaternionAnchors = std::vector<Eigen::Quaterniond,
      Eigen::aligned_allocator<Eigen::Quaterniond>>;
  QuaternionAnchors quaternion_anchors_;
  double quaternion_dt_{0.01};
};

struct TrajectoryLimits
{
  double gravity{9.805}, mass{0.811};
  Eigen::Vector3d inertia{1.91e-3, 2.37e-3, 3.60e-3};
  double arm{0.1216}, arm_angle{0.7853981633974483}, torque_to_thrust{0.01};
  double motor_min{0.0}, motor_max{25.0};
  double thrust_min{0.5}, thrust_max{40.0};
  Eigen::Vector3d rate_max{Eigen::Vector3d::Constant(14.0)};
  double angular_acceleration_max{100.0};
  double minimum_relative_altitude{-0.01};
  bool enforce_minimum_altitude{true};
};
struct TrajectoryAudit
{
  bool valid{true};
  std::string reason;
  double first_failure_time{-1.0};
  double max_speed{0}, max_thrust{0}, max_rate{0}, max_angular_acceleration{0};
  double min_thrust{std::numeric_limits<double>::infinity()};
  double min_motor{std::numeric_limits<double>::infinity()}, max_motor{0};
  double min_altitude{std::numeric_limits<double>::infinity()};
};
// Nominal static allocation only, using the actual project's motor ordering.
// A sampled audit is not a continuous-time or closed-loop feasibility proof.
TrajectoryAudit auditTrajectory(
  const Trajectory & trajectory, const TrajectoryLimits & limits,
  double dt = 0.001);
}  // namespace px4ctrl
