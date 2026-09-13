#include <px4ctrl/barrel_roll_trajectory.h>

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSmall = 1.0e-9;

struct Kinematics
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
};

struct PolynomialSegment
{
  double start_time{0.0};
  double duration{0.0};
  // 第 k 列是局部时间 tau 的 k 次项系数，三个行分别对应 ENU x/y/z。
  Eigen::Matrix<double, 3, 8> coefficients{Eigen::Matrix<double, 3, 8>::Zero()};
};

// 七阶平滑时间律 sigma(s)。其一至三阶导数在两端均为零，使滚转段
// 开始和结束时的横向速度、加速度和 jerk 自然回到零。
std::array<double, 4> septicTimeLaw(double s)
{
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  const double s6 = s5 * s;
  const double s7 = s6 * s;
  return {
    35.0 * s4 - 84.0 * s5 + 70.0 * s6 - 20.0 * s7,
    140.0 * s3 - 420.0 * s4 + 420.0 * s5 - 140.0 * s6,
    420.0 * s2 - 1680.0 * s3 + 2100.0 * s4 - 840.0 * s5,
    840.0 * s - 5040.0 * s2 + 8400.0 * s3 - 4200.0 * s4};
}

double factorialRatio(int power, int derivative)
{
  double result = 1.0;
  for (int i = 0; i < derivative; ++i) {
    result *= static_cast<double>(power - i);
  }
  return result;
}

Eigen::Vector3d evaluate(
  const PolynomialSegment &segment, double local_time, int derivative_order)
{
  const double tau = std::clamp(local_time, 0.0, segment.duration);
  Eigen::Vector3d value = Eigen::Vector3d::Zero();
  for (int power = derivative_order; power <= 7; ++power) {
    value += segment.coefficients.col(power) * factorialRatio(power, derivative_order) *
      std::pow(tau, power - derivative_order);
  }
  return value;
}

// 用两端的 p/v/a/jerk 唯一确定一段七次 Hermite 多项式。
PolynomialSegment makeSegment(
  double start_time, double duration, const Kinematics &start, const Kinematics &end)
{
  PolynomialSegment segment;
  segment.start_time = start_time;
  segment.duration = duration;
  segment.coefficients.col(0) = start.position;
  segment.coefficients.col(1) = start.velocity;
  segment.coefficients.col(2) = 0.5 * start.acceleration;
  segment.coefficients.col(3) = start.jerk / 6.0;

  const double T = duration;
  Eigen::Matrix4d matrix;
  matrix <<
    std::pow(T, 4), std::pow(T, 5), std::pow(T, 6), std::pow(T, 7),
    4.0 * std::pow(T, 3), 5.0 * std::pow(T, 4), 6.0 * std::pow(T, 5),
    7.0 * std::pow(T, 6),
    12.0 * std::pow(T, 2), 20.0 * std::pow(T, 3), 30.0 * std::pow(T, 4),
    42.0 * std::pow(T, 5),
    24.0 * T, 60.0 * std::pow(T, 2), 120.0 * std::pow(T, 3),
    210.0 * std::pow(T, 4);

  const Eigen::Vector3d known_position = start.position + start.velocity * T +
    0.5 * start.acceleration * T * T + start.jerk * std::pow(T, 3) / 6.0;
  const Eigen::Vector3d known_velocity = start.velocity + start.acceleration * T +
    0.5 * start.jerk * T * T;
  const Eigen::Vector3d known_acceleration = start.acceleration + start.jerk * T;
  Eigen::Matrix<double, 4, 3> right_hand_side;
  right_hand_side.row(0) = (end.position - known_position).transpose();
  right_hand_side.row(1) = (end.velocity - known_velocity).transpose();
  right_hand_side.row(2) = (end.acceleration - known_acceleration).transpose();
  right_hand_side.row(3) = (end.jerk - start.jerk).transpose();
  segment.coefficients.rightCols<4>() =
    matrix.fullPivLu().solve(right_hand_side).transpose();
  return segment;
}

Kinematics analyticRollKinematics(
  double roll_time, const Eigen::Vector3d &start_position,
  const Eigen::Vector3d &axis, const Eigen::Vector3d &side,
  const BarrelRollTrajectoryOptions &options)
{
  const double t = std::clamp(roll_time, 0.0, options.roll_duration);
  const auto law = septicTimeLaw(t / options.roll_duration);
  const double angle_scale = 2.0 * kPi * static_cast<double>(options.turns);
  const double theta = angle_scale * law[0];
  const double theta_dot = angle_scale * law[1] / options.roll_duration;
  const double theta_ddot = angle_scale * law[2] /
    (options.roll_duration * options.roll_duration);
  const double theta_dddot = angle_scale * law[3] /
    std::pow(options.roll_duration, 3);
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double radius = options.radius;
  const Eigen::Vector3d roll_start = start_position +
    0.5 * options.axial_speed * options.entry_duration * axis;

  Kinematics result;
  result.position = roll_start + options.axial_speed * t * axis +
    radius * sin_theta * side + radius * (1.0 - cos_theta) * Eigen::Vector3d::UnitZ();
  result.velocity = options.axial_speed * axis +
    radius * cos_theta * theta_dot * side +
    radius * sin_theta * theta_dot * Eigen::Vector3d::UnitZ();
  result.acceleration = radius *
    (-sin_theta * theta_dot * theta_dot + cos_theta * theta_ddot) * side +
    radius * (cos_theta * theta_dot * theta_dot + sin_theta * theta_ddot) *
    Eigen::Vector3d::UnitZ();
  result.jerk = radius *
    (-cos_theta * std::pow(theta_dot, 3) -
    3.0 * sin_theta * theta_dot * theta_ddot + cos_theta * theta_dddot) * side +
    radius * (-sin_theta * std::pow(theta_dot, 3) +
    3.0 * cos_theta * theta_dot * theta_ddot + sin_theta * theta_dddot) *
    Eigen::Vector3d::UnitZ();
  return result;
}

bool appendState(
  const PolynomialSegment &segment, double time, const Eigen::Vector3d &axis,
  double gravity, OmTrajectoryResult *result)
{
  const double local_time = time - segment.start_time;
  const Eigen::Vector3d position = evaluate(segment, local_time, 0);
  const Eigen::Vector3d velocity = evaluate(segment, local_time, 1);
  const Eigen::Vector3d acceleration = evaluate(segment, local_time, 2);
  const Eigen::Vector3d jerk = evaluate(segment, local_time, 3);

  // 微分平坦性映射：a+g*e_z 决定总推力大小和机体 +Z 轴方向。
  const Eigen::Vector3d specific_force = acceleration + gravity * Eigen::Vector3d::UnitZ();
  const double thrust_acceleration = specific_force.norm();
  if (thrust_acceleration < 1.0e-6) {
    result->status = "barrel-roll specific force is too close to zero";
    return false;
  }
  const Eigen::Vector3d b3 = specific_force / thrust_acceleration;
  const Eigen::Vector3d raw_b2 = b3.cross(axis);
  if (raw_b2.norm() < 1.0e-6) {
    result->status = "barrel-roll attitude construction is singular";
    return false;
  }
  const Eigen::Vector3d b2 = raw_b2.normalized();
  const Eigen::Vector3d b1 = b2.cross(b3).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = b1;
  rotation.col(1) = b2;
  rotation.col(2) = b3;

  // jerk 给出推力方向的导数。由 R^T*Rdot 的反对称部分恢复机体系角速度，
  // 使 MPC 的参考输入与参考姿态运动学一致，而不是事后数值差分四元数。
  const Eigen::Vector3d b3_dot =
    (Eigen::Matrix3d::Identity() - b3 * b3.transpose()) * jerk / thrust_acceleration;
  const Eigen::Vector3d b2_dot =
    (Eigen::Matrix3d::Identity() - b2 * b2.transpose()) * (b3_dot.cross(axis)) /
    raw_b2.norm();
  const Eigen::Vector3d b1_dot = b2_dot.cross(b3) + b2.cross(b3_dot);
  Eigen::Matrix3d rotation_dot;
  rotation_dot.col(0) = b1_dot;
  rotation_dot.col(1) = b2_dot;
  rotation_dot.col(2) = b3_dot;
  const Eigen::Matrix3d omega_matrix = 0.5 *
    (rotation.transpose() * rotation_dot - rotation_dot.transpose() * rotation);

  OmTrajectoryState state;
  state.time = time;
  state.position = position;
  state.velocity = velocity;
  state.attitude = Eigen::Quaterniond(rotation).normalized();
  state.thrust_acceleration = thrust_acceleration;
  state.body_rate << omega_matrix(2, 1), omega_matrix(0, 2), omega_matrix(1, 0);
  if (!result->states.empty() &&
    result->states.back().attitude.coeffs().dot(state.attitude.coeffs()) < 0.0)
  {
    // q 和 -q 表示相同姿态；连续符号能保证 setTrajectory() 内的 SLERP
    // 沿相邻采样点的短弧插值，同时仍可累计出完整的 360 度滚转。
    state.attitude.coeffs() *= -1.0;
  }
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
    !state.attitude.coeffs().allFinite() || !state.body_rate.allFinite() ||
    !std::isfinite(state.thrust_acceleration))
  {
    result->status = "barrel-roll trajectory contains a non-finite state";
    return false;
  }
  result->states.push_back(state);
  return true;
}
}  // namespace

OmTrajectoryResult generateBarrelRollTrajectory(
  const Eigen::Vector3d &start_position, const BarrelRollTrajectoryOptions &options)
{
  OmTrajectoryResult result;
  result.status = "barrel-roll trajectory parameters are invalid";
  if (!start_position.allFinite() || !options.roll_axis.allFinite() ||
    options.roll_axis.norm() < kSmall || options.radius <= 0.0 ||
    options.takeoff_height < 0.0 || options.takeoff_duration <= 0.0 ||
    options.takeoff_settle_duration < 0.0 || options.axial_speed < 0.0 ||
    options.entry_duration <= 0.0 ||
    options.roll_duration <= 0.0 || options.exit_duration <= 0.0 ||
    options.turns < 1 || options.polynomial_segments_per_turn < 4 ||
    options.sample_dt <= 0.0 || options.gravity <= 0.0)
  {
    return result;
  }

  const Eigen::Vector3d axis = options.roll_axis.normalized();
  if (std::abs(axis.z()) > 1.0e-6) {
    result.status = "barrel-roll roll_axis must lie in the horizontal plane";
    return result;
  }
  const Eigen::Vector3d side = Eigen::Vector3d::UnitZ().cross(axis).normalized();
  const double maneuver_start_time =
    options.takeoff_duration + options.takeoff_settle_duration;
  const double total_time = maneuver_start_time +
    options.entry_duration + options.roll_duration + options.exit_duration;
  // 后续进入、滚转和退出阶段都以起飞完成的位置为几何起点。
  const Eigen::Vector3d maneuver_start_position =
    start_position + options.takeoff_height * Eigen::Vector3d::UnitZ();
  std::vector<PolynomialSegment> segments;
  segments.reserve(options.turns * options.polynomial_segments_per_turn + 4);

  // 起飞段使用与其他阶段相同的七次 Hermite 构造，首末端 p/v/a/jerk
  // 全部连续。它从当前位置竖直上升，不在尚未离地时引入横向运动或倾角。
  Kinematics takeoff_start;
  takeoff_start.position = start_position;
  Kinematics takeoff_end;
  takeoff_end.position = maneuver_start_position;
  segments.push_back(makeSegment(
    0.0, options.takeoff_duration, takeoff_start, takeoff_end));

  // 起飞后留出短暂稳定时间，使位置和姿态误差在进入水平加速前收敛。
  if (options.takeoff_settle_duration > kSmall) {
    segments.push_back(makeSegment(
      options.takeoff_duration, options.takeoff_settle_duration,
      takeoff_end, takeoff_end));
  }

  // 进入段从静止平滑加速到轴向速度，并在滚转开始前前进半个 v*T。
  Kinematics entry_start;
  entry_start.position = maneuver_start_position;
  Kinematics entry_end;
  entry_end.position = maneuver_start_position +
    0.5 * options.axial_speed * options.entry_duration * axis;
  entry_end.velocity = options.axial_speed * axis;
  segments.push_back(makeSegment(
    maneuver_start_time, options.entry_duration, entry_start, entry_end));

  // 解析螺旋按 p/v/a/jerk 采样，再用七次 Hermite 段连接，保证段间 C3 连续。
  const int roll_segment_count = options.turns * options.polynomial_segments_per_turn;
  const double roll_segment_duration = options.roll_duration / roll_segment_count;
  for (int i = 0; i < roll_segment_count; ++i) {
    const double t0 = i * roll_segment_duration;
    const double t1 = (i + 1) * roll_segment_duration;
    segments.push_back(makeSegment(
      maneuver_start_time + options.entry_duration + t0, roll_segment_duration,
      analyticRollKinematics(t0, maneuver_start_position, axis, side, options),
      analyticRollKinematics(t1, maneuver_start_position, axis, side, options)));
  }

  // 退出段保持位置和速度连续，并从轴向速度平滑减速至静止。
  const Kinematics exit_start = analyticRollKinematics(
    options.roll_duration, maneuver_start_position, axis, side, options);
  Kinematics exit_end;
  exit_end.position = exit_start.position +
    0.5 * options.axial_speed * options.exit_duration * axis;
  segments.push_back(makeSegment(
    maneuver_start_time + options.entry_duration + options.roll_duration,
    options.exit_duration,
    exit_start, exit_end));

  // 重新均分离散时间，保证实际 dt 不大于请求值且最后一点严格落在终点。
  const std::size_t interval_count = static_cast<std::size_t>(
    std::ceil(total_time / options.sample_dt));
  const double actual_dt = total_time / static_cast<double>(interval_count);
  result.states.reserve(interval_count + 1);
  std::size_t segment_index = 0;
  for (std::size_t i = 0; i <= interval_count; ++i) {
    const double time = i * actual_dt;
    while (segment_index + 1 < segments.size() &&
      time >= segments[segment_index + 1].start_time)
    {
      ++segment_index;
    }
    if (!appendState(segments[segment_index], time, axis, options.gravity, &result)) {
      result.states.clear();
      return result;
    }
  }

  result.success = true;
  result.converged = true;
  result.status = "barrel-roll trajectory generated";
  result.total_time = total_time;
  // 记录各阶段边界，便于日志和后续可视化识别起飞、进入、滚转和退出时刻。
  result.waypoint_times = {
    0.0, options.takeoff_duration, maneuver_start_time,
    maneuver_start_time + options.entry_duration,
    maneuver_start_time + options.entry_duration + options.roll_duration,
    total_time};
  return result;
}
