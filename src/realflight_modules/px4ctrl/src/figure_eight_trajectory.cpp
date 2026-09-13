#include <px4ctrl/figure_eight_trajectory.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

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

// 七阶时间律 sigma(s)=35s^4-84s^5+70s^6-20s^7 及其前三阶导数。
// sigma 在 s=0/1 处的一至三阶导数均为零，因此起飞和八字段的 p/v/a/jerk
// 能与两端静止悬停状态连续连接。
std::array<double, 4> septicTimeLaw(double s)
{
  const double value = std::clamp(s, 0.0, 1.0);
  const double s2 = value * value;
  const double s3 = s2 * value;
  const double s4 = s3 * value;
  const double s5 = s4 * value;
  const double s6 = s5 * value;
  const double s7 = s6 * value;
  return {
    35.0 * s4 - 84.0 * s5 + 70.0 * s6 - 20.0 * s7,
    140.0 * s3 - 420.0 * s4 + 420.0 * s5 - 140.0 * s6,
    420.0 * s2 - 1680.0 * s3 + 2100.0 * s4 - 840.0 * s5,
    840.0 * value - 5040.0 * s2 + 8400.0 * s3 - 4200.0 * s4};
}

// 八字段不对整圈使用单个 slow-start 时间律，而只在前后各 15% 时间内平滑
// 加减速，中间保持恒定相位速度。返回归一化相位 phi(s) 及对 s 的前三阶导数。
// 这样既保持 p/v/a/jerk 连续，又避免缓启动把峰值速度放大到平均速度的数倍。
std::array<double, 4> phaseLaw(double s)
{
  constexpr double kRamp = 0.15;
  constexpr double kIntegral = 1.0 - kRamp;
  const double value = std::clamp(s, 0.0, 1.0);
  double integral = 0.0;
  double rate = 1.0;
  double rate_derivative = 0.0;
  double rate_second_derivative = 0.0;
  if (value < kRamp) {
    const double u = value / kRamp;
    const auto law = septicTimeLaw(u);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double u6 = u5 * u;
    const double u7 = u6 * u;
    const double u8 = u7 * u;
    integral = kRamp *
      (7.0 * u5 - 14.0 * u6 + 10.0 * u7 - 2.5 * u8);
    rate = law[0];
    rate_derivative = law[1] / kRamp;
    rate_second_derivative = law[2] / (kRamp * kRamp);
  } else if (value <= 1.0 - kRamp) {
    integral = 0.5 * kRamp + value - kRamp;
  } else {
    const double u = (1.0 - value) / kRamp;
    const auto law = septicTimeLaw(u);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double u6 = u5 * u;
    const double u7 = u6 * u;
    const double u8 = u7 * u;
    const double remaining = kRamp *
      (7.0 * u5 - 14.0 * u6 + 10.0 * u7 - 2.5 * u8);
    integral = kIntegral - remaining;
    rate = law[0];
    rate_derivative = -law[1] / kRamp;
    rate_second_derivative = law[2] / (kRamp * kRamp);
  }
  return {integral / kIntegral, rate / kIntegral,
    rate_derivative / kIntegral, rate_second_derivative / kIntegral};
}

double figureEightSpeedCoefficient(const FigureEightTrajectoryOptions &options)
{
  // 对归一化时间 s 密集采样 max ||dr/dtheta||*(dtheta/ds)。实际速度为
  // coefficient/T，因此令 T=coefficient/options.speed 可保证峰值等于 speed。
  constexpr int kSamples = 16384;
  const double angle_scale = 2.0 * kPi * static_cast<double>(options.laps);
  double maximum = 0.0;
  for (int i = 0; i <= kSamples; ++i) {
    const auto phase = phaseLaw(static_cast<double>(i) / kSamples);
    const double theta = angle_scale * phase[0];
    const double path_derivative = std::hypot(
      0.5 * options.length * std::cos(theta),
      options.width * std::cos(2.0 * theta));
    maximum = std::max(maximum, path_derivative * angle_scale * phase[1]);
  }
  return maximum;
}

Kinematics takeoffKinematics(
  double time, const Eigen::Vector3d &start, const FigureEightTrajectoryOptions &options)
{
  const double duration = options.takeoff_duration;
  const auto law = septicTimeLaw(time / duration);
  Kinematics output;
  output.position = start + options.takeoff_height * law[0] * Eigen::Vector3d::UnitZ();
  output.velocity = options.takeoff_height * law[1] / duration *
    Eigen::Vector3d::UnitZ();
  output.acceleration = options.takeoff_height * law[2] / (duration * duration) *
    Eigen::Vector3d::UnitZ();
  output.jerk = options.takeoff_height * law[3] / std::pow(duration, 3) *
    Eigen::Vector3d::UnitZ();
  return output;
}

Kinematics figureEightKinematics(
  double local_time, double duration, const Eigen::Vector3d &center,
  const Eigen::Vector3d &forward, const Eigen::Vector3d &side,
  const FigureEightTrajectoryOptions &options)
{
  // 相位 theta(t)=2*pi*laps*phi(t/T)。phaseLaw 使 theta_dot、
  // theta_ddot、theta_dddot 在首末端为零，并在中段保持恒定相位速度。
  const auto law = phaseLaw(local_time / duration);
  const double angle_scale = 2.0 * kPi * static_cast<double>(options.laps);
  const double theta = angle_scale * law[0];
  const double theta_dot = angle_scale * law[1] / duration;
  const double theta_ddot = angle_scale * law[2] / (duration * duration);
  const double theta_dddot = angle_scale * law[3] / std::pow(duration, 3);

  // r(theta) 及其对 theta 的前三阶导数。时间导数使用链式法则：
  // v=r'*theta_dot；a=r''*theta_dot^2+r'*theta_ddot；
  // j=r'''*theta_dot^3+3r''*theta_dot*theta_ddot+r'*theta_dddot。
  const Eigen::Vector3d r =
    0.5 * options.length * std::sin(theta) * forward +
    0.5 * options.width * std::sin(2.0 * theta) * side;
  const Eigen::Vector3d r_theta =
    0.5 * options.length * std::cos(theta) * forward +
    options.width * std::cos(2.0 * theta) * side;
  const Eigen::Vector3d r_theta2 =
    -0.5 * options.length * std::sin(theta) * forward -
    2.0 * options.width * std::sin(2.0 * theta) * side;
  const Eigen::Vector3d r_theta3 =
    -0.5 * options.length * std::cos(theta) * forward -
    4.0 * options.width * std::cos(2.0 * theta) * side;

  Kinematics output;
  output.position = center + r;
  output.velocity = r_theta * theta_dot;
  output.acceleration = r_theta2 * theta_dot * theta_dot + r_theta * theta_ddot;
  output.jerk = r_theta3 * std::pow(theta_dot, 3) +
    3.0 * r_theta2 * theta_dot * theta_ddot + r_theta * theta_dddot;
  return output;
}

bool appendState(
  const Kinematics &kinematics, double time, const Eigen::Vector3d &heading,
  double gravity, OmTrajectoryResult *result)
{
  // 微分平坦性映射：f=a+g*e3=a_T*b3，故 a_T=||f||、b3=f/||f||。
  const Eigen::Vector3d specific_force =
    kinematics.acceleration + gravity * Eigen::Vector3d::UnitZ();
  const double thrust_acceleration = specific_force.norm();
  if (thrust_acceleration < 1.0e-6) {
    result->status = "figure-eight specific force is too close to zero";
    return false;
  }
  const Eigen::Vector3d b3 = specific_force / thrust_acceleration;
  const Eigen::Vector3d raw_b2 = b3.cross(heading);
  if (raw_b2.norm() < 1.0e-6) {
    result->status = "figure-eight attitude construction is singular";
    return false;
  }
  const Eigen::Vector3d b2 = raw_b2.normalized();
  const Eigen::Vector3d b1 = b2.cross(b3).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = b1;
  rotation.col(1) = b2;
  rotation.col(2) = b3;

  // 对 f=a+g*e3 求导得到 jerk，再由 b3=f/||f|| 求 b3_dot。
  // R^T*R_dot=[omega]_x，因此其反对称部分的 vee 即机体系角速度参考。
  const Eigen::Vector3d b3_dot =
    (Eigen::Matrix3d::Identity() - b3 * b3.transpose()) *
    kinematics.jerk / thrust_acceleration;
  const Eigen::Vector3d raw_b2_dot = b3_dot.cross(heading);
  const Eigen::Vector3d b2_dot =
    (Eigen::Matrix3d::Identity() - b2 * b2.transpose()) *
    raw_b2_dot / raw_b2.norm();
  const Eigen::Vector3d b1_dot = b2_dot.cross(b3) + b2.cross(b3_dot);
  Eigen::Matrix3d rotation_dot;
  rotation_dot.col(0) = b1_dot;
  rotation_dot.col(1) = b2_dot;
  rotation_dot.col(2) = b3_dot;
  const Eigen::Matrix3d omega_matrix = 0.5 *
    (rotation.transpose() * rotation_dot - rotation_dot.transpose() * rotation);

  OmTrajectoryState state;
  state.time = time;
  state.position = kinematics.position;
  state.velocity = kinematics.velocity;
  state.attitude = Eigen::Quaterniond(rotation).normalized();
  state.thrust_acceleration = thrust_acceleration;
  state.body_rate << omega_matrix(2, 1), omega_matrix(0, 2), omega_matrix(1, 0);
  if (!result->states.empty() &&
    result->states.back().attitude.coeffs().dot(state.attitude.coeffs()) < 0.0)
  {
    state.attitude.coeffs() *= -1.0;
  }
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
    !state.attitude.coeffs().allFinite() || !state.body_rate.allFinite() ||
    !std::isfinite(state.thrust_acceleration))
  {
    result->status = "figure-eight trajectory contains a non-finite state";
    return false;
  }
  result->states.push_back(state);
  return true;
}
}  // namespace

OmTrajectoryResult generateFigureEightTrajectory(
  const Eigen::Vector3d &start_position, const FigureEightTrajectoryOptions &options)
{
  OmTrajectoryResult result;
  result.status = "figure-eight trajectory parameters are invalid";
  if (!start_position.allFinite() || !options.forward_axis.allFinite() ||
    options.forward_axis.norm() < kSmall || options.takeoff_height < 0.0 ||
    options.takeoff_duration <= 0.0 || options.takeoff_settle_duration < 0.0 ||
    options.length <= 0.0 || options.width <= 0.0 || options.speed <= 0.0 ||
    options.laps < 1 || options.sample_dt <= 0.0 || options.gravity <= 0.0)
  {
    return result;
  }

  Eigen::Vector3d forward = options.forward_axis;
  forward.z() = 0.0;
  if (forward.norm() < kSmall) {
    result.status = "figure-eight forward_axis must have a horizontal component";
    return result;
  }
  forward.normalize();
  const Eigen::Vector3d side = Eigen::Vector3d::UnitZ().cross(forward).normalized();
  const Eigen::Vector3d center =
    start_position + options.takeoff_height * Eigen::Vector3d::UnitZ();

  // speed 是最大路径速度。速度关于总时间成反比，先计算 T=1 时的峰值系数，
  // 再反求 T_8，使整段 ||v(t)||<=speed。
  const double figure_duration = figureEightSpeedCoefficient(options) / options.speed;
  const double figure_start = options.takeoff_duration + options.takeoff_settle_duration;
  const double total_time = figure_start + figure_duration;
  const std::size_t interval_count = static_cast<std::size_t>(
    std::ceil(total_time / options.sample_dt));
  if (interval_count == 0) {
    return result;
  }
  const double actual_dt = total_time / static_cast<double>(interval_count);
  result.states.reserve(interval_count + 1);

  for (std::size_t i = 0; i <= interval_count; ++i) {
    const double time = static_cast<double>(i) * actual_dt;
    Kinematics kinematics;
    if (time < options.takeoff_duration) {
      kinematics = takeoffKinematics(time, start_position, options);
    } else if (time < figure_start) {
      kinematics.position = center;
    } else {
      kinematics = figureEightKinematics(
        time - figure_start, figure_duration, center, forward, side, options);
    }
    if (!appendState(kinematics, time, forward, options.gravity, &result)) {
      result.states.clear();
      return result;
    }
  }

  result.success = true;
  result.converged = true;
  result.status = "takeoff + figure-eight trajectory generated";
  result.total_time = total_time;
  result.waypoint_times = {
    0.0, options.takeoff_duration, figure_start, total_time};
  return result;
}
