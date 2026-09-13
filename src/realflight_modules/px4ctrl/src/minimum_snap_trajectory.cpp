#include <px4ctrl/minimum_snap_trajectory.h>

#include <Eigen/Geometry>
#include <Eigen/QR>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{
constexpr int kCoefficientCount = 8;
constexpr int kHighestContinuousDerivative = 3;

double derivativeFactor(int power, int derivative)
{
  double factor = 1.0;
  for (int i = 0; i < derivative; ++i) {
    factor *= static_cast<double>(power - i);
  }
  return factor;
}

double basisDerivative(int power, int derivative, double tau, double duration)
{
  if (power < derivative) {
    return 0.0;
  }
  return derivativeFactor(power, derivative) *
    std::pow(tau, power - derivative) / std::pow(duration, derivative);
}

Eigen::Vector3d evaluatePolynomial(
  const Eigen::MatrixXd &coefficients, int segment, double tau,
  double duration, int derivative)
{
  Eigen::Vector3d value = Eigen::Vector3d::Zero();
  const int offset = segment * kCoefficientCount;
  for (int power = derivative; power < kCoefficientCount; ++power) {
    value += basisDerivative(power, derivative, tau, duration) *
      coefficients.row(offset + power).transpose();
  }
  return value;
}

Eigen::Matrix3d attitudeFromForce(const Eigen::Vector3d &specific_force, double yaw)
{
  Eigen::Vector3d body_z = specific_force;
  if (!body_z.allFinite() || body_z.norm() < 1.0e-8) {
    body_z = Eigen::Vector3d::UnitZ();
  } else {
    body_z.normalize();
  }
  const Eigen::Vector3d heading_y(-std::sin(yaw), std::cos(yaw), 0.0);
  Eigen::Vector3d body_x = heading_y.cross(body_z);
  if (body_x.norm() < 1.0e-8) {
    body_x = Eigen::Vector3d::UnitX();
  } else {
    body_x.normalize();
  }
  Eigen::Vector3d body_y = body_z.cross(body_x);
  if (body_y.norm() < 1.0e-8) {
    body_y = Eigen::Vector3d::UnitY();
  } else {
    body_y.normalize();
  }
  Eigen::Matrix3d rotation;
  rotation.col(0) = body_x;
  rotation.col(1) = body_y;
  rotation.col(2) = body_z;
  return rotation;
}

Eigen::Vector3d logSo3(const Eigen::Matrix3d &rotation)
{
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1.0e-10) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.angle() * angle_axis.axis();
}
}  // namespace

MinimumSnapTrajectory::MinimumSnapTrajectory(
  std::size_t point_count, const std::vector<Eigen::Vector3d> &points,
  const MinimumSnapOptions &options)
: point_count_(point_count), points_(points), options_(options)
{
}

OmTrajectoryResult MinimumSnapTrajectory::generate() const
{
  const auto solve_start = std::chrono::steady_clock::now();
  OmTrajectoryResult result;
  const auto fail = [&](const std::string &message) {
      result.status = "minimum snap: " + message;
      result.total_solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - solve_start).count();
      return result;
    };

  if (point_count_ != points_.size()) {
    return fail("point_count does not match points.size()");
  }
  if (point_count_ < 2U) {
    return fail("at least two points are required");
  }
  if (!(options_.nominal_speed > 0.0) || !(options_.minimum_segment_time > 0.0) ||
    !(options_.sample_dt > 0.0) || !(options_.gravity > 0.0))
  {
    return fail("speed, segment time, sample_dt and gravity must be positive");
  }
  for (const auto &point : points_) {
    if (!point.allFinite()) {
      return fail("all waypoint coordinates must be finite");
    }
  }

  const int segment_count = static_cast<int>(point_count_ - 1U);
  std::vector<double> durations(static_cast<std::size_t>(segment_count));
  std::vector<double> knot_times(point_count_, 0.0);
  for (int segment = 0; segment < segment_count; ++segment) {
    const double distance = (points_[static_cast<std::size_t>(segment + 1)] -
      points_[static_cast<std::size_t>(segment)]).norm();
    durations[static_cast<std::size_t>(segment)] = std::max(
      options_.minimum_segment_time, distance / options_.nominal_speed);
    knot_times[static_cast<std::size_t>(segment + 1)] =
      knot_times[static_cast<std::size_t>(segment)] +
      durations[static_cast<std::size_t>(segment)];
  }

  const int variable_count = kCoefficientCount * segment_count;
  const int constraint_count = 2 * segment_count +
    kHighestContinuousDerivative * (segment_count - 1) +
    2 * kHighestContinuousDerivative;
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variable_count, variable_count);
  for (int segment = 0; segment < segment_count; ++segment) {
    const double duration = durations[static_cast<std::size_t>(segment)];
    for (int row = 4; row < kCoefficientCount; ++row) {
      for (int col = 4; col < kCoefficientCount; ++col) {
        hessian(segment * kCoefficientCount + row,
          segment * kCoefficientCount + col) =
          derivativeFactor(row, 4) * derivativeFactor(col, 4) /
          (static_cast<double>(row + col - 7) * std::pow(duration, 7));
      }
    }
  }
  // Snap cost has a polynomial nullspace. Equality constraints remove the
  // physical ambiguity; this tiny diagonal only improves numerical rank tests.
  hessian.diagonal().array() += 1.0e-12;

  Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(constraint_count, variable_count);
  Eigen::MatrixXd targets = Eigen::MatrixXd::Zero(constraint_count, 3);
  int constraint_row = 0;
  for (int segment = 0; segment < segment_count; ++segment) {
    const double duration = durations[static_cast<std::size_t>(segment)];
    for (int power = 0; power < kCoefficientCount; ++power) {
      constraints(constraint_row, segment * kCoefficientCount + power) =
        basisDerivative(power, 0, 0.0, duration);
    }
    targets.row(constraint_row++) = points_[static_cast<std::size_t>(segment)].transpose();
    for (int power = 0; power < kCoefficientCount; ++power) {
      constraints(constraint_row, segment * kCoefficientCount + power) =
        basisDerivative(power, 0, 1.0, duration);
    }
    targets.row(constraint_row++) = points_[static_cast<std::size_t>(segment + 1)].transpose();
  }
  for (int segment = 0; segment + 1 < segment_count; ++segment) {
    const double current_duration = durations[static_cast<std::size_t>(segment)];
    const double next_duration = durations[static_cast<std::size_t>(segment + 1)];
    for (int derivative = 1; derivative <= kHighestContinuousDerivative; ++derivative) {
      for (int power = 0; power < kCoefficientCount; ++power) {
        constraints(constraint_row, segment * kCoefficientCount + power) =
          basisDerivative(power, derivative, 1.0, current_duration);
        constraints(constraint_row, (segment + 1) * kCoefficientCount + power) =
          -basisDerivative(power, derivative, 0.0, next_duration);
      }
      ++constraint_row;
    }
  }
  for (int derivative = 1; derivative <= kHighestContinuousDerivative; ++derivative) {
    for (int power = 0; power < kCoefficientCount; ++power) {
      constraints(constraint_row, power) = basisDerivative(
        power, derivative, 0.0, durations.front());
    }
    ++constraint_row;
  }
  for (int derivative = 1; derivative <= kHighestContinuousDerivative; ++derivative) {
    for (int power = 0; power < kCoefficientCount; ++power) {
      constraints(constraint_row, (segment_count - 1) * kCoefficientCount + power) =
        basisDerivative(power, derivative, 1.0, durations.back());
    }
    ++constraint_row;
  }

  Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(
    variable_count + constraint_count, variable_count + constraint_count);
  kkt.topLeftCorner(variable_count, variable_count) = hessian;
  kkt.topRightCorner(variable_count, constraint_count) = constraints.transpose();
  kkt.bottomLeftCorner(constraint_count, variable_count) = constraints;
  Eigen::MatrixXd right_hand_side = Eigen::MatrixXd::Zero(
    variable_count + constraint_count, 3);
  right_hand_side.bottomRows(constraint_count) = targets;
  const Eigen::MatrixXd solution = kkt.completeOrthogonalDecomposition().solve(right_hand_side);
  if (!solution.allFinite()) {
    return fail("polynomial solve produced non-finite coefficients");
  }
  const Eigen::MatrixXd coefficients = solution.topRows(variable_count);
  const double equality_residual =
    (constraints * coefficients - targets).cwiseAbs().maxCoeff();
  if (equality_residual > 1.0e-6) {
    return fail("polynomial equality residual is " + std::to_string(equality_residual));
  }

  result.total_time = knot_times.back();
  result.waypoint_times = knot_times;
  const int sample_intervals = std::max(
    1, static_cast<int>(std::ceil(result.total_time / options_.sample_dt)));
  const double actual_dt = result.total_time / static_cast<double>(sample_intervals);
  result.states.reserve(static_cast<std::size_t>(sample_intervals + 1));
  int segment = 0;
  for (int sample_index = 0; sample_index <= sample_intervals; ++sample_index) {
    const double time = sample_index == sample_intervals ?
      result.total_time : sample_index * actual_dt;
    while (segment + 1 < segment_count &&
      time > knot_times[static_cast<std::size_t>(segment + 1)] + 1.0e-12)
    {
      ++segment;
    }
    const double local_time = time - knot_times[static_cast<std::size_t>(segment)];
    const double duration = durations[static_cast<std::size_t>(segment)];
    const double tau = std::clamp(local_time / duration, 0.0, 1.0);
    OmTrajectoryState state;
    state.time = time;
    state.position = evaluatePolynomial(coefficients, segment, tau, duration, 0);
    state.velocity = evaluatePolynomial(coefficients, segment, tau, duration, 1);
    const Eigen::Vector3d acceleration =
      evaluatePolynomial(coefficients, segment, tau, duration, 2);
    const Eigen::Vector3d specific_force = acceleration +
      Eigen::Vector3d(0.0, 0.0, options_.gravity);
    state.thrust_acceleration = specific_force.norm();
    state.attitude = Eigen::Quaterniond(
      attitudeFromForce(specific_force, options_.yaw)).normalized();
    result.states.push_back(state);
  }
  // omega_k is the constant body rate that rotates R_k to R_{k+1} over the
  // cached sample interval. This matches the MPC discrete attitude model.
  for (int k = 0; k < sample_intervals; ++k) {
    const Eigen::Matrix3d current = result.states[static_cast<std::size_t>(k)].attitude
      .toRotationMatrix();
    const Eigen::Matrix3d next = result.states[static_cast<std::size_t>(k + 1)].attitude
      .toRotationMatrix();
    result.states[static_cast<std::size_t>(k)].body_rate =
      logSo3(current.transpose() * next) / actual_dt;
  }
  result.states.back().body_rate = sample_intervals > 0 ?
    result.states[result.states.size() - 2U].body_rate : Eigen::Vector3d::Zero();

  result.success = true;
  result.converged = true;
  result.status = "minimum snap polynomial trajectory";
  result.total_solve_time = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - solve_start).count();
  return result;
}
