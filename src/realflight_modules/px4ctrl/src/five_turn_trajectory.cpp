#include <px4ctrl/five_turn_trajectory.h>

#include <Eigen/LU>
#include <Eigen/QR>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace px4ctrl
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSmall = 1.0e-9;

double factorialRatio(int power, int derivative)
{
  double value = 1.0;
  for (int i = 0; i < derivative; ++i) {
    value *= static_cast<double>(power - i);
  }
  return value;
}

double angleBetween(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  const double cosine = std::clamp(
    first.normalized().dot(second.normalized()), -1.0, 1.0);
  return std::acos(cosine);
}

double unwrapIncrement(double current, double previous)
{
  double increment = current - previous;
  while (increment > kPi) {increment -= 2.0 * kPi;}
  while (increment < -kPi) {increment += 2.0 * kPi;}
  return increment;
}

// 入口和出口的灰色几何种子使用五阶 smoothstep，避免可视化参考线出现折角。
double smoothStep(double s)
{
  const double clamped = std::clamp(s, 0.0, 1.0);
  return clamped * clamped * clamped *
         (10.0 - 15.0 * clamped + 6.0 * clamped * clamped);
}

// 与旧 BarrelRollHelixTrajectory 完全相同的七阶时间律。前三阶导数在
// 两端为零，因此位置相位、加速度方向和姿态不会在翻滚入口/出口突变。
std::array<double, 4> septicTimeLaw(double s)
{
  const double x = std::clamp(s, 0.0, 1.0);
  const double x2 = x * x;
  const double x3 = x2 * x;
  const double x4 = x3 * x;
  const double x5 = x4 * x;
  const double x6 = x5 * x;
  const double x7 = x6 * x;
  return {
    35.0 * x4 - 84.0 * x5 + 70.0 * x6 - 20.0 * x7,
    140.0 * x3 - 420.0 * x4 + 420.0 * x5 - 140.0 * x6,
    420.0 * x2 - 1680.0 * x3 + 2100.0 * x4 - 840.0 * x5,
    840.0 * x - 5040.0 * x2 + 8400.0 * x3 - 4200.0 * x4};
}
}  // namespace

int FiveTurnHelixTrajectory::coefficientIndex(
  int segment, int axis, int power) const
{
  return (segment * kDimension + axis) * kCoefficientCount + power;
}

Eigen::Matrix<double, FiveTurnHelixTrajectory::kCoefficientCount, 1>
FiveTurnHelixTrajectory::derivativeBasis(
  int derivative_order, double local_time, double duration) const
{
  Eigen::Matrix<double, kCoefficientCount, 1> basis;
  basis.setZero();
  const double normalized_time = std::clamp(local_time / duration, 0.0, 1.0);
  const double time_scale = std::pow(duration, -derivative_order);
  for (int power = derivative_order; power < kCoefficientCount; ++power) {
    basis[power] = factorialRatio(power, derivative_order) *
      std::pow(normalized_time, power - derivative_order) * time_scale;
  }
  return basis;
}

double FiveTurnHelixTrajectory::segmentDuration(int segment) const
{
  return knot_times_[segment + 1] - knot_times_[segment];
}

double FiveTurnHelixTrajectory::knotTime(int knot) const
{
  return knot_times_[std::clamp(knot, 0, segment_count_)];
}

// 对七阶时间律做单调二分反解，使每个圆形路标的时刻与旧节点完全一致。
double FiveTurnHelixTrajectory::rollLocalTimeFromPhase(double phase) const
{
  const double total_phase = 2.0 * kPi * static_cast<double>(parameters_.revolutions);
  const double target = std::clamp(phase / total_phase, 0.0, 1.0);
  double lower = 0.0;
  double upper = 1.0;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (septicTimeLaw(middle)[0] < target) {
      lower = middle;
    } else {
      upper = middle;
    }
  }
  return 0.5 * (lower + upper) * parameters_.roll_duration;
}

double FiveTurnHelixTrajectory::rollPhaseFromLocalTime(double roll_local_time) const
{
  const double clamped = std::clamp(roll_local_time, 0.0, parameters_.roll_duration);
  const double normalized_time = clamped / parameters_.roll_duration;
  return 2.0 * kPi * static_cast<double>(parameters_.revolutions) *
         septicTimeLaw(normalized_time)[0];
}

FiveTurnHelixTrajectory::SeedKinematics
FiveTurnHelixTrajectory::analyticRollKinematics(double roll_local_time) const
{
  const double t = std::clamp(roll_local_time, 0.0, parameters_.roll_duration);
  const double normalized_time = t / parameters_.roll_duration;
  const auto law = septicTimeLaw(normalized_time);
  const double angle_scale = 2.0 * kPi * static_cast<double>(parameters_.revolutions);
  const double theta = angle_scale * law[0];
  const double theta_dot = angle_scale * law[1] / parameters_.roll_duration;
  const double theta_ddot = angle_scale * law[2] /
    std::pow(parameters_.roll_duration, 2);
  const double theta_dddot = angle_scale * law[3] /
    std::pow(parameters_.roll_duration, 3);
  const double sine = std::sin(theta);
  const double cosine = std::cos(theta);
  const double radius = parameters_.radius;

  SeedKinematics result;
  result.position = rollPosition(t, theta);
  result.velocity = parameters_.axial_speed * roll_axis_ +
    radius * cosine * theta_dot * lateral_axis_ +
    radius * sine * theta_dot * Eigen::Vector3d::UnitZ();
  result.acceleration = radius *
    (-sine * theta_dot * theta_dot + cosine * theta_ddot) * lateral_axis_ +
    radius *
    (cosine * theta_dot * theta_dot + sine * theta_ddot) *
    Eigen::Vector3d::UnitZ();
  result.jerk = radius *
    (-cosine * std::pow(theta_dot, 3) -
    3.0 * sine * theta_dot * theta_ddot + cosine * theta_dddot) * lateral_axis_ +
    radius *
    (-sine * std::pow(theta_dot, 3) +
    3.0 * cosine * theta_dot * theta_ddot + sine * theta_dddot) *
    Eigen::Vector3d::UnitZ();
  return result;
}

Eigen::Vector3d FiveTurnHelixTrajectory::rollPosition(
  double roll_local_time, double phase) const
{
  const Eigen::Vector3d roll_start = parameters_.start_position +
    0.5 * parameters_.axial_speed * parameters_.entry_duration * roll_axis_;
  return roll_start + parameters_.axial_speed * roll_local_time * roll_axis_ +
    parameters_.radius * std::sin(phase) * lateral_axis_ +
    parameters_.radius * (1.0 - std::cos(phase)) * Eigen::Vector3d::UnitZ();
}

Eigen::Vector3d FiveTurnHelixTrajectory::waypoint(int knot) const
{
  const int clamped = std::clamp(knot, 0, segment_count_);
  if (clamped == 0) {
    return parameters_.start_position;
  }
  if (clamped <= roll_segment_count_ + 1) {
    const double roll_local_time = knot_times_[clamped] - parameters_.entry_duration;
    return rollPosition(roll_local_time, knot_phases_[clamped]);
  }
  const Eigen::Vector3d roll_finish =
    rollPosition(parameters_.roll_duration, 2.0 * kPi * parameters_.revolutions);
  return roll_finish +
    0.5 * parameters_.axial_speed * parameters_.exit_duration * roll_axis_;
}

Eigen::Vector3d FiveTurnHelixTrajectory::desiredThrustDirection(int knot) const
{
  const int clamped = std::clamp(knot, 0, segment_count_);
  const double roll_local_time = std::clamp(
    knot_times_[clamped] - parameters_.entry_duration,
    0.0, parameters_.roll_duration);
  const Eigen::Vector3d specific_force =
    analyticRollKinematics(roll_local_time).acceleration +
    parameters_.gravity * Eigen::Vector3d::UnitZ();
  return specific_force.normalized();
}

bool FiveTurnHelixTrajectory::isAttitudeKnot(int knot) const
{
  if (knot < 1 || knot > roll_segment_count_ + 1) {
    return false;
  }
  const int roll_knot = knot - 1;
  const int per_revolution = parameters_.circle_waypoints_per_revolution;
  const int phase_index = roll_knot % per_revolution;
  return phase_index == 0 ||
         phase_index == per_revolution / 4 ||
         phase_index == per_revolution / 2 ||
         phase_index == 3 * per_revolution / 4;
}

int FiveTurnHelixTrajectory::segmentForTime(double time) const
{
  const double clamped = std::clamp(time, 0.0, total_duration_);
  if (clamped >= total_duration_) {
    return segment_count_ - 1;
  }
  const auto upper = std::upper_bound(knot_times_.begin(), knot_times_.end(), clamped);
  return std::clamp(
    static_cast<int>(std::distance(knot_times_.begin(), upper)) - 1,
    0, segment_count_ - 1);
}

Eigen::Vector3d FiveTurnHelixTrajectory::evaluateSegment(
  int segment, double local_time, int derivative_order) const
{
  const double tau = std::clamp(local_time, 0.0, segmentDuration(segment));
  const auto basis = derivativeBasis(
    derivative_order, tau, segmentDuration(segment));
  Eigen::Vector3d value = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < kDimension; ++axis) {
    for (int power = 0; power < kCoefficientCount; ++power) {
      value[axis] += coefficients_(
        coefficientIndex(segment, axis, power), 0) * basis[power];
    }
  }
  return value;
}

Eigen::Vector3d FiveTurnHelixTrajectory::evaluate(
  double time, int derivative_order) const
{
  const double clamped = std::clamp(time, 0.0, total_duration_);
  const int segment = segmentForTime(clamped);
  return evaluateSegment(segment, clamped - knot_times_[segment], derivative_order);
}

bool FiveTurnHelixTrajectory::generate(
  const FiveTurnHelixParameters & parameters, std::string * error)
{
  generated_ = false;
  auto fail = [&](const std::string & message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
  if (!parameters.start_position.allFinite() || !parameters.roll_axis.allFinite()) {
    return fail("起点或滚转轴包含非有限值");
  }
  if (parameters.roll_axis.norm() < kSmall ||
    std::abs(parameters.roll_axis.normalized().z()) > 1.0e-6)
  {
    return fail("滚转/前进轴必须是非零水平向量");
  }
  if (parameters.revolutions < 1 || parameters.revolutions > 20 ||
    parameters.circle_waypoints_per_revolution < 8 ||
    parameters.circle_waypoints_per_revolution % 4 != 0 ||
    parameters.radius <= 0.0 || parameters.axial_speed < 0.0 ||
    parameters.entry_duration <= 0.0 || parameters.roll_duration <= 0.0 ||
    parameters.exit_duration <= 0.0 || parameters.mass <= 0.0 ||
    parameters.gravity <= 0.0 ||
    !std::isfinite(parameters.velocity_tracking_weight) ||
    !std::isfinite(parameters.acceleration_tracking_weight) ||
    !std::isfinite(parameters.jerk_tracking_weight) ||
    parameters.velocity_tracking_weight < 0.0 ||
    parameters.acceleration_tracking_weight < 0.0 ||
    parameters.jerk_tracking_weight < 0.0)
  {
    return fail("圈数、圆周路标数、几何参数、时长、跟踪权重、质量或重力非法");
  }

  parameters_ = parameters;
  roll_axis_ = parameters.roll_axis.normalized();
  lateral_axis_ = Eigen::Vector3d::UnitZ().cross(roll_axis_).normalized();
  roll_segment_count_ = parameters.revolutions *
    parameters.circle_waypoints_per_revolution;
  segment_count_ = roll_segment_count_ + 2;
  total_duration_ = parameters.entry_duration + parameters.roll_duration +
    parameters.exit_duration;

  // 节点0是入口起点；节点1..M+1是圆形翻滚段的 M+1 个路标；最后节点
  // 是出口终点。改进版使用与旧节点相同的全局七阶相位及其反函数定时。
  knot_times_.assign(segment_count_ + 1, 0.0);
  knot_phases_.assign(segment_count_ + 1, 0.0);
  knot_times_[0] = 0.0;
  for (int j = 0; j <= roll_segment_count_; ++j) {
    const double phase = 2.0 * kPi * static_cast<double>(j) /
      static_cast<double>(parameters.circle_waypoints_per_revolution);
    knot_phases_[j + 1] = phase;
    knot_times_[j + 1] = parameters.entry_duration + rollLocalTimeFromPhase(phase);
  }
  knot_times_[segment_count_] = total_duration_;
  knot_phases_[segment_count_] =
    2.0 * kPi * static_cast<double>(parameters.revolutions);

  const int variable_count = segment_count_ * kDimension * kCoefficientCount;
  const int attitude_knot_count = 4 * parameters.revolutions + 1;
  const int equality_count = 18 + 15 * (segment_count_ - 1) +
    3 * attitude_knot_count;
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variable_count, variable_count);
  for (int segment = 0; segment < segment_count_; ++segment) {
    const double h = segmentDuration(segment);
    if (h <= kSmall) {
      return fail("圆周节点时刻没有严格递增");
    }
    for (int axis = 0; axis < kDimension; ++axis) {
      for (int i = 3; i < kCoefficientCount; ++i) {
        for (int j = 3; j < kCoefficientCount; ++j) {
          hessian(
            coefficientIndex(segment, axis, i),
            coefficientIndex(segment, axis, j)) +=
            2.0 * factorialRatio(i, 3) * factorialRatio(j, 3) *
            std::pow(h, -5) / static_cast<double>(i + j - 5);
        }
      }
    }
  }
  const Eigen::MatrixXd jerk_hessian = hessian;
  Eigen::VectorXd linear_term = Eigen::VectorXd::Zero(variable_count);

  // 上一版只固定位置和少数姿态，加速度方向在路标之间仍可回绕。改进版将
  // 同一七阶圆形种子的 v/a/jerk 作为软目标加入QP；它们不破坏硬约束，
  // 但会在剩余自由度中选择与旧丝滑轨迹导数最接近的解。
  auto add_derivative_tracking = [&](int segment, int derivative, int axis,
      double target, double weight) {
      if (weight <= 0.0) {
        return;
      }
      const auto basis = derivativeBasis(
        derivative, 0.0, segmentDuration(segment));
      for (int i = 0; i < kCoefficientCount; ++i) {
        const int row_index = coefficientIndex(segment, axis, i);
        linear_term[row_index] -= 2.0 * weight * basis[i] * target;
        for (int j = 0; j < kCoefficientCount; ++j) {
          hessian(row_index, coefficientIndex(segment, axis, j)) +=
            2.0 * weight * basis[i] * basis[j];
        }
      }
    };
  for (int knot = 1; knot <= roll_segment_count_ + 1; ++knot) {
    const int right_segment = knot;
    const SeedKinematics seed = analyticRollKinematics(
      knot_times_[knot] - parameters_.entry_duration);
    for (int axis = 0; axis < kDimension; ++axis) {
      add_derivative_tracking(
        right_segment, 1, axis, seed.velocity[axis],
        parameters_.velocity_tracking_weight);
      add_derivative_tracking(
        right_segment, 2, axis, seed.acceleration[axis],
        parameters_.acceleration_tracking_weight);
      add_derivative_tracking(
        right_segment, 3, axis, seed.jerk[axis],
        parameters_.jerk_tracking_weight);
    }
  }

  Eigen::MatrixXd equality = Eigen::MatrixXd::Zero(equality_count, variable_count);
  Eigen::VectorXd right_hand_side = Eigen::VectorXd::Zero(equality_count);
  int row = 0;
  auto add_axis_constraint = [&](int segment, double local_time,
      int derivative, int axis, double value) {
      const auto basis = derivativeBasis(
        derivative, local_time, segmentDuration(segment));
      for (int power = 0; power < kCoefficientCount; ++power) {
        equality(row, coefficientIndex(segment, axis, power)) = basis[power];
      }
      right_hand_side[row] = value;
      ++row;
    };

  const Eigen::Vector3d start = waypoint(0);
  const Eigen::Vector3d finish = waypoint(segment_count_);
  const double final_h = segmentDuration(segment_count_ - 1);
  for (int axis = 0; axis < kDimension; ++axis) {
    add_axis_constraint(0, 0.0, 0, axis, start[axis]);
    add_axis_constraint(0, 0.0, 1, axis, 0.0);
    add_axis_constraint(0, 0.0, 2, axis, 0.0);
    add_axis_constraint(segment_count_ - 1, final_h, 0, axis, finish[axis]);
    add_axis_constraint(segment_count_ - 1, final_h, 1, axis, 0.0);
    add_axis_constraint(segment_count_ - 1, final_h, 2, axis, 0.0);
  }

  for (int knot = 1; knot < segment_count_; ++knot) {
    const int left = knot - 1;
    const int right = knot;
    const double left_h = segmentDuration(left);
    for (int derivative = 0; derivative <= 3; ++derivative) {
      const auto left_basis = derivativeBasis(derivative, left_h, left_h);
      const auto right_basis = derivativeBasis(
        derivative, 0.0, segmentDuration(right));
      for (int axis = 0; axis < kDimension; ++axis) {
        for (int power = 0; power < kCoefficientCount; ++power) {
          equality(row, coefficientIndex(left, axis, power)) = left_basis[power];
          equality(row, coefficientIndex(right, axis, power)) = -right_basis[power];
        }
        ++row;
      }
    }
    const Eigen::Vector3d position = waypoint(knot);
    for (int axis = 0; axis < kDimension; ++axis) {
      add_axis_constraint(right, 0.0, 0, axis, position[axis]);
    }

    // 四个主相位的期望推力方向来自同一解析圆形种子的加速度。概念上，
    // 其中两个正交分量就是论文的姿态方向约束；第三个沿d的分量固定正推力
    // 幅值，消除上一版 d/-d 二义性。三者合起来等价于 a=a_seed，仍是线性
    // 等式，且不再与圆形运动自身的动力学冲突。
    if (isAttitudeKnot(knot)) {
      const Eigen::Vector3d desired_acceleration =
        analyticRollKinematics(
        knot_times_[knot] - parameters_.entry_duration).acceleration;
      for (int axis = 0; axis < kDimension; ++axis) {
        add_axis_constraint(right, 0.0, 2, axis, desired_acceleration[axis]);
      }
    }

  }
  if (row != equality_count) {
    return fail("内部错误：圆形五圈 QP 约束计数不一致");
  }

  // 直接解KKT时，重复圈产生的对称约束会让拉格朗日乘子表示病态。这里改用
  // 零空间法：先求 A*c0=b 的可行解和 A 的零空间 Z，再在 c=c0+Z*z 上
  // 最小化 jerk。它与KKT最优条件等价，但只需求解一个很小的约化QP。
  const auto equality_cod = equality.completeOrthogonalDecomposition();
  const Eigen::VectorXd feasible = equality_cod.solve(right_hand_side);
  const double feasible_residual =
    (equality * feasible - right_hand_side).lpNorm<Eigen::Infinity>();
  if (!feasible.allFinite() || feasible_residual > 1.0e-6) {
    return fail(
      "圆形五圈线性约束不相容，残差：" + std::to_string(feasible_residual));
  }
  const Eigen::MatrixXd nullspace = equality.fullPivLu().kernel();
  Eigen::VectorXd optimized = feasible;
  if (nullspace.cols() > 0) {
    const Eigen::MatrixXd reduced_hessian =
      nullspace.transpose() * hessian * nullspace;
    const Eigen::VectorXd reduced_rhs =
      -nullspace.transpose() * (hessian * feasible + linear_term);
    const Eigen::VectorXd reduced_solution =
      reduced_hessian.completeOrthogonalDecomposition().solve(reduced_rhs);
    optimized += nullspace * reduced_solution;
  }
  if (!optimized.allFinite()) {
    return fail("圆形五圈最小 jerk QP 求解产生非有限值");
  }
  coefficients_ = optimized;
  equality_residual_ =
    (equality * coefficients_ - right_hand_side).lpNorm<Eigen::Infinity>();
  integrated_squared_jerk_ =
    0.5 * coefficients_.col(0).dot(jerk_hessian * coefficients_.col(0));
  if (equality_residual_ > 1.0e-6) {
    return fail(
      "圆形五圈最小 jerk QP 等式残差过大：" +
      std::to_string(equality_residual_));
  }
  generated_ = true;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

FiveTurnSample FiveTurnHelixTrajectory::sample(double time) const
{
  FiveTurnSample result;
  if (!generated_) {
    return result;
  }
  result.time = std::clamp(time, 0.0, total_duration_);
  result.progress = result.time / total_duration_;
  result.position = evaluate(result.time, 0);
  result.velocity = evaluate(result.time, 1);
  result.acceleration = evaluate(result.time, 2);
  result.jerk = evaluate(result.time, 3);
  result.snap = evaluate(result.time, 4);

  const Eigen::Vector3d specific_force = result.acceleration +
    parameters_.gravity * Eigen::Vector3d::UnitZ();
  const double force_norm = specific_force.norm();
  result.thrust = parameters_.mass * force_norm;
  if (force_norm < 1.0e-6) {
    return result;
  }
  const Eigen::Vector3d b3 = specific_force / force_norm;
  const Eigen::Vector3d raw_b2 = b3.cross(roll_axis_);
  if (raw_b2.norm() < 1.0e-6) {
    return result;
  }
  const Eigen::Vector3d b2 = raw_b2.normalized();
  const Eigen::Vector3d b1 = b2.cross(b3).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = b1;
  rotation.col(1) = b2;
  rotation.col(2) = b3;
  result.orientation = Eigen::Quaterniond(rotation).normalized();

  const Eigen::Vector3d b3_dot =
    (Eigen::Matrix3d::Identity() - b3 * b3.transpose()) * result.jerk / force_norm;
  const Eigen::Vector3d b2_dot =
    (Eigen::Matrix3d::Identity() - b2 * b2.transpose()) *
    (b3_dot.cross(roll_axis_)) / raw_b2.norm();
  const Eigen::Vector3d b1_dot = b2_dot.cross(b3) + b2.cross(b3_dot);
  Eigen::Matrix3d rotation_dot;
  rotation_dot.col(0) = b1_dot;
  rotation_dot.col(1) = b2_dot;
  rotation_dot.col(2) = b3_dot;
  const Eigen::Matrix3d omega_hat = 0.5 *
    (rotation.transpose() * rotation_dot - rotation_dot.transpose() * rotation);
  result.body_rate << omega_hat(2, 1), omega_hat(0, 2), omega_hat(1, 0);
  result.valid = result.position.allFinite() && result.velocity.allFinite() &&
    result.acceleration.allFinite() && result.jerk.allFinite() &&
    result.snap.allFinite() && result.orientation.coeffs().allFinite() &&
    result.body_rate.allFinite() && std::isfinite(result.thrust);
  return result;
}

DiscretizedFiveTurnTrajectory FiveTurnHelixTrajectory::discretize(
  double requested_dt) const
{
  DiscretizedFiveTurnTrajectory result;
  result.requested_dt = requested_dt;
  if (!generated_ || requested_dt <= 0.0) {
    return result;
  }
  const std::size_t intervals = static_cast<std::size_t>(
    std::ceil(total_duration_ / requested_dt));
  result.actual_dt = total_duration_ / static_cast<double>(intervals);
  result.points.reserve(intervals + 1);
  for (std::size_t i = 0; i <= intervals; ++i) {
    FiveTurnSample point = sample(static_cast<double>(i) * result.actual_dt);
    if (!result.points.empty() &&
      result.points.back().orientation.coeffs().dot(point.orientation.coeffs()) < 0.0)
    {
      point.orientation.coeffs() *= -1.0;
    }
    result.points.push_back(point);
  }
  return result;
}

Eigen::Vector3d FiveTurnHelixTrajectory::analyticPosition(double time) const
{
  if (!generated_) {
    return Eigen::Vector3d::Zero();
  }
  const double clamped = std::clamp(time, 0.0, total_duration_);
  const Eigen::Vector3d roll_start = parameters_.start_position +
    0.5 * parameters_.axial_speed * parameters_.entry_duration * roll_axis_;
  if (clamped < parameters_.entry_duration) {
    return parameters_.start_position + smoothStep(clamped / parameters_.entry_duration) *
      (roll_start - parameters_.start_position);
  }
  if (clamped <= rollEndTime()) {
    const double local = clamped - parameters_.entry_duration;
    return rollPosition(local, rollPhaseFromLocalTime(local));
  }
  const Eigen::Vector3d roll_finish =
    rollPosition(parameters_.roll_duration, 2.0 * kPi * parameters_.revolutions);
  const Eigen::Vector3d finish = waypoint(segment_count_);
  return roll_finish + smoothStep(
    (clamped - rollEndTime()) / parameters_.exit_duration) * (finish - roll_finish);
}

FiveTurnHelixValidationReport FiveTurnHelixTrajectory::validate(
  double check_dt) const
{
  FiveTurnHelixValidationReport report;
  report.qp_equality_residual = equality_residual_;
  report.integrated_squared_jerk = integrated_squared_jerk_;
  if (!generated_ || check_dt <= 0.0) {
    report.warnings.emplace_back("轨迹尚未生成，或验证步长不是正数");
    return report;
  }

  report.minimum_thrust = std::numeric_limits<double>::infinity();
  report.minimum_altitude = std::numeric_limits<double>::infinity();
  report.maximum_altitude = -std::numeric_limits<double>::infinity();
  const std::size_t count = static_cast<std::size_t>(
    std::ceil(total_duration_ / check_dt));
  bool all_valid = true;
  bool have_phase = false;
  double previous_position_phase = 0.0;
  double previous_attitude_phase = 0.0;
  const Eigen::Vector3d circle_center_offset =
    parameters_.radius * Eigen::Vector3d::UnitZ();

  for (std::size_t i = 0; i <= count; ++i) {
    const double time = total_duration_ * static_cast<double>(i) /
      static_cast<double>(count);
    const FiveTurnSample point = sample(time);
    all_valid = all_valid && point.valid;
    report.minimum_thrust = std::min(report.minimum_thrust, point.thrust);
    report.maximum_thrust = std::max(report.maximum_thrust, point.thrust);
    report.maximum_speed = std::max(report.maximum_speed, point.velocity.norm());
    report.maximum_acceleration = std::max(
      report.maximum_acceleration, point.acceleration.norm());
    report.maximum_jerk = std::max(report.maximum_jerk, point.jerk.norm());
    report.maximum_body_rate = std::max(
      report.maximum_body_rate, point.body_rate.norm());
    report.minimum_altitude = std::min(report.minimum_altitude, point.position.z());
    report.maximum_altitude = std::max(report.maximum_altitude, point.position.z());

    if (time >= rollStartTime() && time <= rollEndTime() && point.valid) {
      report.maximum_circle_seed_error = std::max(
        report.maximum_circle_seed_error,
        (point.position - analyticPosition(time)).norm());
      const double roll_local = time - rollStartTime();
      const Eigen::Vector3d moving_center = parameters_.start_position +
        0.5 * parameters_.axial_speed * parameters_.entry_duration * roll_axis_ +
        parameters_.axial_speed * roll_local * roll_axis_ + circle_center_offset;
      const Eigen::Vector3d radial = point.position - moving_center;
      const double position_phase = std::atan2(
        radial.dot(lateral_axis_), -radial.z());
      const Eigen::Vector3d b3 = point.orientation * Eigen::Vector3d::UnitZ();
      const double attitude_phase = std::atan2(-b3.dot(lateral_axis_), b3.z());
      if (!have_phase) {
        previous_position_phase = position_phase;
        previous_attitude_phase = attitude_phase;
        have_phase = true;
      } else {
        const double position_increment =
          unwrapIncrement(position_phase, previous_position_phase) * 180.0 / kPi;
        const double attitude_increment =
          unwrapIncrement(attitude_phase, previous_attitude_phase) * 180.0 / kPi;
        report.position_winding_angle_deg += position_increment;
        report.attitude_winding_angle_deg += attitude_increment;
        if (attitude_increment < 0.0) {
          report.maximum_attitude_phase_backtracking_deg = std::max(
            report.maximum_attitude_phase_backtracking_deg, -attitude_increment);
        }
        previous_position_phase = position_phase;
        previous_attitude_phase = attitude_phase;
      }
    }
  }

  for (int knot = 0; knot <= segment_count_; ++knot) {
    const FiveTurnSample point = sample(knotTime(knot));
    report.maximum_waypoint_error = std::max(
      report.maximum_waypoint_error, (point.position - waypoint(knot)).norm());
    if (isAttitudeKnot(knot)) {
      report.maximum_attitude_constraint_error_deg = std::max(
        report.maximum_attitude_constraint_error_deg,
        angleBetween(
          point.orientation * Eigen::Vector3d::UnitZ(),
          desiredThrustDirection(knot)) * 180.0 / kPi);
    }
  }

  for (int connection = 0; connection < segment_count_ - 1; ++connection) {
    const double h = segmentDuration(connection);
    for (int derivative = 0; derivative <= 3; ++derivative) {
      const double connection_error =
        (evaluateSegment(connection, h, derivative) -
        evaluateSegment(connection + 1, 0.0, derivative)).norm();
      if (derivative == 0) {
        report.maximum_connection_position_error = std::max(
          report.maximum_connection_position_error, connection_error);
      } else if (derivative == 1) {
        report.maximum_connection_velocity_error = std::max(
          report.maximum_connection_velocity_error, connection_error);
      } else if (derivative == 2) {
        report.maximum_connection_acceleration_error = std::max(
          report.maximum_connection_acceleration_error, connection_error);
      } else {
        report.maximum_connection_jerk_error = std::max(
          report.maximum_connection_jerk_error, connection_error);
      }
    }
  }
  report.axial_advance =
    (sample(total_duration_).position - sample(0.0).position).dot(roll_axis_);

  const double expected_winding = 360.0 * parameters_.revolutions;
  if (!all_valid) {
    report.warnings.emplace_back("存在零推力或姿态构造奇异的样本");
  }
  if (report.qp_equality_residual > 1.0e-6) {
    report.warnings.emplace_back("QP 等式约束残差超过容差");
  }
  if (report.maximum_waypoint_error > 1.0e-6) {
    report.warnings.emplace_back("圆形位置路标误差超过容差");
  }
  if (report.maximum_attitude_constraint_error_deg > 0.05) {
    report.warnings.emplace_back("论文姿态约束出现方向反号或数值误差");
  }
  if (std::abs(report.position_winding_angle_deg - expected_winding) > 5.0) {
    report.warnings.emplace_back("位置没有完成要求的五个圆周");
  }
  if (std::abs(report.attitude_winding_angle_deg - expected_winding) > 5.0) {
    report.warnings.emplace_back("姿态没有完成要求的五圈翻滚");
  }
  if (report.maximum_attitude_phase_backtracking_deg > 0.5) {
    report.warnings.emplace_back("姿态相位存在明显局部回绕，视觉上可能不丝滑");
  }
  // 默认每圈16个圆周路标，段内相对解析圆形的偏离应明显小于半径。
  if (report.maximum_circle_seed_error > 0.20 * parameters_.radius) {
    report.warnings.emplace_back("QP 曲线在圆周路标之间偏离圆形种子过大");
  }
  if (report.minimum_thrust < 0.01 * parameters_.mass * parameters_.gravity) {
    report.warnings.emplace_back("轨迹出现接近零推力的区间");
  }
  if (report.minimum_altitude < 0.0) {
    report.warnings.emplace_back("轨迹穿过 z=0");
  }
  if (report.maximum_connection_position_error > 1.0e-6 ||
    report.maximum_connection_velocity_error > 1.0e-6 ||
    report.maximum_connection_acceleration_error > 1.0e-5 ||
    report.maximum_connection_jerk_error > 1.0e-4)
  {
    report.warnings.emplace_back("多项式段连接误差超过 C3 数值容差");
  }
  report.valid = report.warnings.empty();
  return report;
}

namespace
{
struct AdapterKinematics
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
};

AdapterKinematics takeoffKinematics(
  double time, const Eigen::Vector3d &start,
  const FiveTurnTrajectoryOptions &options)
{
  const auto law = septicTimeLaw(time / options.takeoff_duration);
  AdapterKinematics result;
  result.position = start + options.takeoff_height * law[0] * Eigen::Vector3d::UnitZ();
  result.velocity = options.takeoff_height * law[1] / options.takeoff_duration *
    Eigen::Vector3d::UnitZ();
  result.acceleration = options.takeoff_height * law[2] /
    std::pow(options.takeoff_duration, 2) * Eigen::Vector3d::UnitZ();
  result.jerk = options.takeoff_height * law[3] /
    std::pow(options.takeoff_duration, 3) * Eigen::Vector3d::UnitZ();
  return result;
}

bool appendAdapterState(
  const AdapterKinematics &kinematics, double time,
  const Eigen::Vector3d &heading, double gravity, OmTrajectoryResult *result)
{
  const Eigen::Vector3d specific_force =
    kinematics.acceleration + gravity * Eigen::Vector3d::UnitZ();
  const double thrust_acceleration = specific_force.norm();
  if (thrust_acceleration < 1.0e-6) {
    result->status = "five-turn takeoff specific force is too close to zero";
    return false;
  }
  const Eigen::Vector3d b3 = specific_force / thrust_acceleration;
  const Eigen::Vector3d raw_b2 = b3.cross(heading);
  if (raw_b2.norm() < 1.0e-6) {
    result->status = "five-turn takeoff attitude construction is singular";
    return false;
  }
  const Eigen::Vector3d b2 = raw_b2.normalized();
  const Eigen::Vector3d b1 = b2.cross(b3).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = b1;
  rotation.col(1) = b2;
  rotation.col(2) = b3;

  const Eigen::Vector3d b3_dot =
    (Eigen::Matrix3d::Identity() - b3 * b3.transpose()) *
    kinematics.jerk / thrust_acceleration;
  const Eigen::Vector3d b2_dot =
    (Eigen::Matrix3d::Identity() - b2 * b2.transpose()) *
    (b3_dot.cross(heading)) / raw_b2.norm();
  const Eigen::Vector3d b1_dot = b2_dot.cross(b3) + b2.cross(b3_dot);
  Eigen::Matrix3d rotation_dot;
  rotation_dot.col(0) = b1_dot;
  rotation_dot.col(1) = b2_dot;
  rotation_dot.col(2) = b3_dot;
  const Eigen::Matrix3d omega_hat = 0.5 *
    (rotation.transpose() * rotation_dot - rotation_dot.transpose() * rotation);

  OmTrajectoryState state;
  state.time = time;
  state.position = kinematics.position;
  state.velocity = kinematics.velocity;
  state.attitude = Eigen::Quaterniond(rotation).normalized();
  state.thrust_acceleration = thrust_acceleration;
  state.body_rate << omega_hat(2, 1), omega_hat(0, 2), omega_hat(1, 0);
  if (!result->states.empty() &&
    result->states.back().attitude.coeffs().dot(state.attitude.coeffs()) < 0.0)
  {
    state.attitude.coeffs() *= -1.0;
  }
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
    !state.attitude.coeffs().allFinite() || !state.body_rate.allFinite() ||
    !std::isfinite(state.thrust_acceleration))
  {
    result->status = "five-turn takeoff contains a non-finite state";
    return false;
  }
  result->states.push_back(state);
  return true;
}

bool appendManeuverState(
  const FiveTurnSample &sample, double shifted_time, double mass,
  OmTrajectoryResult *result)
{
  if (!sample.valid || mass <= 0.0) {
    result->status = "five-turn QP produced an invalid maneuver sample";
    return false;
  }
  OmTrajectoryState state;
  state.time = shifted_time;
  state.position = sample.position;
  state.velocity = sample.velocity;
  state.attitude = sample.orientation;
  state.thrust_acceleration = sample.thrust / mass;
  state.body_rate = sample.body_rate;
  if (!result->states.empty() &&
    result->states.back().attitude.coeffs().dot(state.attitude.coeffs()) < 0.0)
  {
    state.attitude.coeffs() *= -1.0;
  }
  result->states.push_back(state);
  return true;
}
}  // namespace

OmTrajectoryResult generateFiveTurnTrajectory(
  const Eigen::Vector3d &start_position,
  const FiveTurnTrajectoryOptions &options)
{
  OmTrajectoryResult result;
  result.status = "five-turn trajectory parameters are invalid";
  if (!start_position.allFinite() || !options.roll_axis.allFinite() ||
    options.roll_axis.norm() < kSmall || options.takeoff_height < 0.0 ||
    options.takeoff_duration <= 0.0 || options.takeoff_settle_duration < 0.0 ||
    options.sample_dt <= 0.0 || options.mass <= 0.0 || options.gravity <= 0.0)
  {
    return result;
  }

  Eigen::Vector3d roll_axis = options.roll_axis;
  roll_axis.z() = 0.0;
  if (roll_axis.norm() < kSmall) {
    result.status = "five-turn roll axis must have a horizontal component";
    return result;
  }
  roll_axis.normalize();

  FiveTurnHelixParameters parameters;
  parameters.start_position =
    start_position + options.takeoff_height * Eigen::Vector3d::UnitZ();
  parameters.roll_axis = roll_axis;
  parameters.revolutions = options.revolutions;
  parameters.circle_waypoints_per_revolution = options.circle_waypoints_per_revolution;
  parameters.radius = options.radius;
  parameters.axial_speed = options.axial_speed;
  parameters.entry_duration = options.entry_duration;
  parameters.roll_duration = options.roll_duration;
  parameters.exit_duration = options.exit_duration;
  parameters.velocity_tracking_weight = options.velocity_tracking_weight;
  parameters.acceleration_tracking_weight = options.acceleration_tracking_weight;
  parameters.jerk_tracking_weight = options.jerk_tracking_weight;
  parameters.mass = options.mass;
  parameters.gravity = options.gravity;

  FiveTurnHelixTrajectory maneuver;
  std::string generation_error;
  if (!maneuver.generate(parameters, &generation_error)) {
    result.status = "five-turn QP failed: " + generation_error;
    return result;
  }
  const FiveTurnHelixValidationReport validation =
    maneuver.validate(std::max(0.001, 0.5 * options.sample_dt));
  if (!validation.valid) {
    result.status = "five-turn validation failed";
    if (!validation.warnings.empty()) {
      result.status += ": " + validation.warnings.front();
    }
    return result;
  }

  const double maneuver_start =
    options.takeoff_duration + options.takeoff_settle_duration;
  const double total_time = maneuver_start + maneuver.duration();
  const std::size_t interval_count = static_cast<std::size_t>(
    std::ceil(total_time / options.sample_dt));
  const double actual_dt = total_time / static_cast<double>(interval_count);
  result.states.reserve(interval_count + 1);
  for (std::size_t i = 0; i <= interval_count; ++i) {
    const double time = static_cast<double>(i) * actual_dt;
    if (time < options.takeoff_duration) {
      if (!appendAdapterState(
          takeoffKinematics(time, start_position, options), time,
          roll_axis, options.gravity, &result))
      {
        result.states.clear();
        return result;
      }
    } else if (time < maneuver_start) {
      AdapterKinematics settle;
      settle.position = parameters.start_position;
      if (!appendAdapterState(
          settle, time, roll_axis, options.gravity, &result))
      {
        result.states.clear();
        return result;
      }
    } else {
      const double maneuver_time = std::min(time - maneuver_start, maneuver.duration());
      if (!appendManeuverState(
          maneuver.sample(maneuver_time), time, options.mass, &result))
      {
        result.states.clear();
        return result;
      }
    }
  }

  result.success = true;
  result.converged = true;
  result.status = "five-turn minimum-jerk trajectory generated";
  result.total_time = total_time;
  result.waypoint_times = {
    0.0,
    options.takeoff_duration,
    maneuver_start,
    maneuver_start + parameters.entry_duration,
    maneuver_start + parameters.entry_duration + parameters.roll_duration,
    total_time};
  return result;
}

}  // namespace px4ctrl
