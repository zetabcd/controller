#include <px4ctrl/trajectory.h>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace px4ctrl
{
namespace
{
constexpr double pi = 3.14159265358979323846;
double factor(int n, int d)
{
  double f = 1; for (int i = 0; i < d; ++i) {
    f *= n - i;
  }
  return f;
}
std::array<double, 5> smooth(double u)
{
  u = std::clamp(u, 0.0, 1.0);
  const std::array<double, 9> c{0, 0, 0, 0, 0, 7, -14, 10, -2.5}; // integral of H
  std::array<double, 5> out{};
  for (int d = 0; d <= 4; ++d) {
    for (int k = d; k <= 8; ++k) {
      out[d] += c[k] * factor(k, d) * std::pow(u, k - d);
    }
  }
  return out;
}
std::array<double, 5> phase(double t, double ramp, double cruise, double rate)
{
  std::array<double, 5> q{};
  if (t < ramp) {
    q = smooth(t / ramp);
    for (int d = 0; d <= 4; ++d) {
      q[d] *= rate * std::pow(ramp, 1 - d);
    }
  } else if (t <= ramp + cruise) {
    q[0] = rate * (0.5 * ramp + t - ramp); q[1] = rate;
  } else {
    const double u = std::clamp((t - ramp - cruise) / ramp, 0.0, 1.0);
    q = smooth(u);
    q[0] = rate * (0.5 * ramp + cruise + ramp * (u - q[0]));
    q[1] = rate * (1 - q[1]);
    for (int d = 2; d <= 4; ++d) {
      q[d] *= -rate * std::pow(ramp, 1 - d);
    }
  }
  return q;
}
ReferencePoint chain(const std::array<Eigen::Vector3d, 5> & p, const std::array<double, 5> & q)
{
  ReferencePoint r;
  const double w = q[1], a = q[2], j = q[3], s = q[4];
  r.position = p[0];r.velocity = p[1] * w;
  r.acceleration = p[2] * w * w + p[1] * a;
  r.jerk = p[3] * w * w * w + 3 * p[2] * w * a + p[1] * j;
  r.snap = p[4] * std::pow(w, 4) + 6 * p[3] * w * w * a + p[2] * (3 * a * a + 4 * w * j) + p[1] * s;
  return r;
}
}

void TrajectoryPlayer::start(
  std::shared_ptr<const Trajectory> source, double stamp,
  const Eigen::Vector3d & origin, double yaw)
{
  if (!source || !std::isfinite(stamp) || !origin.allFinite() || !std::isfinite(yaw)) {
    throw std::invalid_argument("Invalid trajectory activation");
  }
  source_ = std::move(source);start_ = last_stamp_ = stamp;origin_ = origin;
  rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}
void TrajectoryPlayer::clear() {source_.reset();}
ReferenceWindow TrajectoryPlayer::sample(double now, int horizon, double dt)
{
  if (!source_ || !std::isfinite(now) || now < last_stamp_ - 1e-6 || horizon < 0 ||
    !std::isfinite(dt) || dt <= 0)
  {
    throw std::invalid_argument("Inactive trajectory or invalid/backwards clock");
  }
  ReferenceWindow w{now, dt, {}};w.points.reserve(horizon + 1);
  for (int k = 0; k <= horizon; ++k) {
    auto r = source_->evaluate(now - start_ + k * dt);
    r.position = origin_ + rotation_ * r.position;
    r.velocity = rotation_ * r.velocity;r.acceleration = rotation_ * r.acceleration;
    r.jerk = rotation_ * r.jerk;r.snap = rotation_ * r.snap;
    r.attitude = (rotation_ * r.attitude).normalized();
    const auto R = r.attitude.toRotationMatrix();r.yaw = std::atan2(R(1, 0), R(0, 0));
    // Body vectors do not change under a rotation of the world frame.
    if (!w.points.empty() && w.points.back().attitude.dot(r.attitude) < 0) {
      r.attitude.coeffs() *= -1;
    }
    w.points.push_back(r);
  }
  validateReferenceWindow(w, horizon, dt);
  last_stamp_ = now;
  return w;
}

AnalyticTrajectory::Polynomial AnalyticTrajectory::connect(
  const ReferencePoint & a, const ReferencePoint & b, double duration)
{
  Eigen::Matrix<double, 10, 10> matrix = Eigen::Matrix<double, 10, 10>::Zero();
  Eigen::Matrix<double, 10, 3> rhs;
  const std::array<Eigen::Vector3d, 5> left{a.position, a.velocity, a.acceleration, a.jerk, a.snap};
  const std::array<Eigen::Vector3d, 5> right{b.position, b.velocity, b.acceleration, b.jerk,
    b.snap};
  for (int d = 0; d <= 4; ++d) {
    matrix(d, d) = factor(d, d);
    for (int k = d; k <= 9; ++k) {
      matrix(d + 5, k) = factor(k, d);
    }
    rhs.row(d) = left[d].transpose() * std::pow(duration, d);
    rhs.row(d + 5) = right[d].transpose() * std::pow(duration, d);
  }
  return matrix.partialPivLu().solve(rhs).transpose();
}
ReferencePoint AnalyticTrajectory::polynomial(const Polynomial & c, double t, double duration)
{
  std::array<Eigen::Vector3d, 5> values;
  const double u = std::clamp(t / duration, 0.0, 1.0);
  for (int d = 0; d <= 4; ++d) {
    values[d].setZero();
    for (int k = d; k <= 9; ++k) {
      values[d] += c.col(k) * factor(k, d) * std::pow(u, k - d) / std::pow(duration, d);
    }
  }
  ReferencePoint r;r.position = values[0];r.velocity = values[1];r.acceleration = values[2];
  r.jerk = values[3];r.snap = values[4];return r;
}

AnalyticTrajectory::AnalyticTrajectory(const AnalyticTrajectoryOptions & o)
: options_(o)
{
  for (double v : {o.gravity, o.takeoff_height, o.takeoff_duration, o.settle_duration, o.radius,
      o.speed, o.ramp_duration, o.centripetal_g, o.pitch, o.entry_duration, o.exit_duration,
      o.entry_distance, o.exit_distance, o.connector_height, o.axis_transition_duration,
      o.eight_length, o.eight_width})
  {
    if (!std::isfinite(v)) {throw std::invalid_argument("Trajectory parameters must be finite");}
  }
  if (o.gravity <= 0 || o.takeoff_height < 0 || o.takeoff_duration <= 0 || o.settle_duration < 0 ||
    o.radius <= 0 || o.radius > 1 || o.turns < 1 || o.turns > 20 || o.speed <= 0 ||
    o.ramp_duration <= 0 ||
    o.pitch < 0 || o.entry_duration <= 0 || o.exit_duration <= 0 || o.entry_distance <= 0 ||
    o.exit_distance <= 0 || o.connector_height < 0 || o.axis_transition_duration <= 0 ||
    o.eight_length <= 0 || o.eight_width <= 0)
  {throw std::invalid_argument("Invalid trajectory geometry/timing (circle radius must be <=1 m)");}
  ReferencePoint zero, hover;hover.position.z() = o.takeoff_height;
  takeoff_ = connect(zero, hover, o.takeoff_duration);
  const double ready = o.takeoff_duration + o.settle_duration;
  main_origin_ = hover.position;
  const bool flip = o.path == AnalyticPath::VerticalCircle || o.path == AnalyticPath::Helix;
  if (flip) {
    if (o.centripetal_g <= 1) {
      throw std::invalid_argument("Inverted circle requires centripetal_g > 1");
    }
    phase_rate_ = std::sqrt(o.centripetal_g * o.gravity / o.radius);
    main_origin_ += o.entry_distance * (o.path == AnalyticPath::VerticalCircle ?
      Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY());
    main_origin_.z() -= o.connector_height;
    const bool helix = o.path == AnalyticPath::Helix;
    const double axis_time = helix ? o.axis_transition_duration : 0.0;
    const double axis_speed = helix ? o.pitch * phase_rate_ / (2 * pi) : 0.0;
    ReferencePoint entry_begin = hover;
    if (helix) {
      entry_begin.position.x() = 0.5 * axis_speed * axis_time;
      entry_begin.velocity.x() = axis_speed;
      axis_entry_ = connect(hover, entry_begin, axis_time);
      main_origin_.x() = axis_speed * (0.5 * axis_time + o.entry_duration);
    }
    entry_start_ = ready + axis_time;
    main_start_ = entry_start_ + o.entry_duration;
    main_end_ = main_start_ + 2 * pi * o.turns / phase_rate_;
    entry_ = connect(entry_begin, main(0), o.entry_duration);
    const auto end = main(main_end_ - main_start_);
    ReferencePoint exit_end;exit_end.position = end.position + o.exit_distance *
      (o.path ==
      AnalyticPath::VerticalCircle ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY());
    exit_end.position.z() += o.connector_height;
    exit_end.position.x() += axis_speed * o.exit_duration;
    exit_end.velocity.x() = axis_speed;
    exit_ = connect(end, exit_end, o.exit_duration);
    exit_end_ = main_end_ + o.exit_duration;
    ReferencePoint hold;hold.position = exit_end.position;
    hold.position.x() += 0.5 * axis_speed * axis_time;
    if (helix) {axis_exit_ = connect(exit_end, hold, axis_time);}
    end_ = hold.position;
    boundaries_ = {0, o.takeoff_duration, ready};
    if (helix) {boundaries_.push_back(entry_start_);}
    boundaries_.push_back(main_start_);boundaries_.push_back(main_end_);
    boundaries_.push_back(exit_end_);
    if (helix) {boundaries_.push_back(exit_end_ + axis_time);}
  } else {
    main_start_ = ready;
    const double derivative_bound = o.path == AnalyticPath::HorizontalCircle ? o.radius :
      std::hypot(0.5 * o.eight_length, o.eight_width);
    phase_rate_ = o.speed / derivative_bound;
    cruise_duration_ = 2 * pi * o.turns / phase_rate_;
    // Eight closes exactly; horizontal circle follows the document's cruise-turn convention.
    if (o.path == AnalyticPath::FigureEight) {cruise_duration_ -= o.ramp_duration;}
    if (cruise_duration_ < 0) {
      throw std::invalid_argument(
              "Eight too short for requested speed/ramp; increase turns or reduce speed");
    }
    main_end_ = main_start_ + 2 * o.ramp_duration + cruise_duration_;
    end_ = main(main_end_ - main_start_).position;
    boundaries_ = {0, o.takeoff_duration, ready, ready + o.ramp_duration,
      ready + o.ramp_duration + cruise_duration_, main_end_};
  }
  if (duration() > 600) {throw std::invalid_argument("Trajectory exceeds 600 s allocation budget");}
  // Only lift quaternion signs; positions and all derivatives remain analytic.
  const int count = static_cast<int>(std::ceil(duration() / 0.01));
  quaternion_dt_ = duration() / count;quaternion_anchors_.reserve(count + 1);
  for (int i = 0; i <= count; ++i) {
    auto q = evaluate(i * quaternion_dt_).attitude;
    if (!quaternion_anchors_.empty() && quaternion_anchors_.back().dot(q) < 0) {q.coeffs() *= -1;}
    quaternion_anchors_.push_back(q);
  }
}

ReferencePoint AnalyticTrajectory::main(double t) const
{
  const auto & o = options_;
  const bool flip = o.path == AnalyticPath::VerticalCircle || o.path == AnalyticPath::Helix;
  std::array<double, 5> q = flip ? std::array<double, 5>{phase_rate_ * t, phase_rate_, 0, 0, 0} :
  phase(t, o.ramp_duration, cruise_duration_, phase_rate_);
  std::array<Eigen::Vector3d, 5> p;
  for (auto & v:p) {
    v.setZero();
  }
  const double theta = q[0], r = o.radius;
  for (int d = 0; d <= 4; ++d) {
    const double sn = std::sin(theta + d * pi / 2), cs = std::cos(theta + d * pi / 2);
    switch (o.path) {
      case AnalyticPath::HorizontalCircle: p[d] = {r * cs, r * sn, 0};break;
      case AnalyticPath::VerticalCircle: p[d] = {r * sn, 0, -r * cs};break;
      case AnalyticPath::Helix: p[d] = {0, r * sn, -r * cs};break;
      case AnalyticPath::FigureEight:
        p[d] = {0.5 * o.eight_length * sn, 0.5 * o.eight_width * std::pow(2, d) *
          std::sin(2 * theta + d * pi / 2), 0};break;
    }
  }
  if (o.path == AnalyticPath::HorizontalCircle) {p[0].x() -= r;}
  if (flip) {p[0].z() += r;}
  if (o.path == AnalyticPath::Helix) {
    p[0].x() = o.pitch * theta / (2 * pi);p[1].x() = o.pitch / (2 * pi);
  }
  p[0] += main_origin_;
  return chain(p, q);
}

ReferencePoint AnalyticTrajectory::evaluate(double t) const
{
  if (!std::isfinite(t)) {throw std::invalid_argument("Nonfinite trajectory time");}
  ReferencePoint r;
  const auto & o = options_;
  const double ready = o.takeoff_duration + o.settle_duration;
  if (t <= 0) {r.position.setZero();} else if (t >= duration()) {
    r.position = end_;
  } else if (t < o.takeoff_duration) {
    r = polynomial(takeoff_, t, o.takeoff_duration);
  } else if (t < ready) {r.position.z() = o.takeoff_height;} else if (t < entry_start_) {
    r = polynomial(axis_entry_, t - ready, o.axis_transition_duration);
  } else if (t < main_start_) {
    r = polynomial(entry_, t - entry_start_, o.entry_duration);
  } else if (t <= main_end_) {r = main(t - main_start_);} else if (t < exit_end_) {
    r = polynomial(exit_, t - main_end_, o.exit_duration);
  } else {r = polynomial(axis_exit_, t - exit_end_, o.axis_transition_duration);}
  const bool vertical = o.path == AnalyticPath::VerticalCircle;
  r = resolveGeometricReference(
    r, o.gravity, vertical ? Eigen::Vector3d::UnitY() :
    Eigen::Vector3d::UnitX(), vertical ? HeadingAxis::BodyY : HeadingAxis::BodyX);
  if (!quaternion_anchors_.empty()) {
    const std::size_t index = std::min(
      quaternion_anchors_.size() - 1,
      static_cast<std::size_t>(std::clamp(t, 0.0, duration()) / quaternion_dt_));
    if (quaternion_anchors_[index].dot(r.attitude) < 0) {r.attitude.coeffs() *= -1;}
  }
  return r;
}

TrajectoryAudit auditTrajectory(
  const Trajectory & trajectory, const TrajectoryLimits & l,
  double dt)
{
  if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(trajectory.duration()) ||
    trajectory.duration() <= 0 || trajectory.duration() / dt > 2000000 ||
    !std::isfinite(
      l.mass) || l.mass <= 0 || !l.inertia.allFinite() || (l.inertia.array() <= 0).any() ||
    !std::isfinite(l.arm) || l.arm <= 0 || !std::isfinite(l.arm_angle) ||
    !std::isfinite(l.torque_to_thrust) || l.torque_to_thrust <= 0 ||
    !std::isfinite(l.motor_min) || !std::isfinite(l.motor_max) || l.motor_max <= l.motor_min ||
    !std::isfinite(l.thrust_min) || !std::isfinite(l.thrust_max) || l.thrust_min <= 0 ||
    l.thrust_max <= l.thrust_min ||
    !l.rate_max.allFinite() || (l.rate_max.array() <= 0).any() ||
    !std::isfinite(l.angular_acceleration_max) || l.angular_acceleration_max <= 0 ||
    !std::isfinite(l.minimum_relative_altitude))
  {throw std::invalid_argument("Invalid trajectory audit bounds");}
  Eigen::Matrix4d B;
  const double a = l.arm * std::cos(l.arm_angle), b = l.arm * std::sin(l.arm_angle),
    k = l.torque_to_thrust;
  B << 1, 1, 1, 1, b, -b, -b, b, -a, -a, a, a, k, -k, k, -k;
  if (std::abs(B.determinant()) < 1e-12) {
    throw std::invalid_argument("Singular allocation matrix");
  }
  const Eigen::Matrix4d mix = B.inverse();
  TrajectoryAudit out;
  const auto fail = [&](double t, const std::string & reason) {
      if (out.valid) {out.valid = false;out.first_failure_time = t;out.reason = reason;}
    };
  std::vector<double> times = trajectory.boundaries();
  const int n = static_cast<int>(std::ceil(trajectory.duration() / dt));
  for (int i = 0; i <= n; ++i) {
    times.push_back(trajectory.duration() * i / n);
  }
  std::sort(times.begin(), times.end());
  for (double t:times) {
    try {
      const auto r = trajectory.evaluate(t);
      validateReferenceWindow({t, dt, {r}}, 0, dt);
      out.max_speed = std::max(out.max_speed, r.velocity.norm());
      out.max_thrust = std::max(out.max_thrust, r.thrust_acceleration);
      out.min_thrust = std::min(out.min_thrust, r.thrust_acceleration);
      out.max_rate = std::max(out.max_rate, r.body_rate.norm());
      out.min_altitude = std::min(out.min_altitude, r.position.z());
      if (r.thrust_acceleration < l.thrust_min || r.thrust_acceleration > l.thrust_max) {
        fail(t, "thrust acceleration limit");
      }
      if ((r.body_rate.cwiseAbs().array() > l.rate_max.array()).any()) {fail(t, "body rate limit");}
      if (r.position.z() < l.minimum_relative_altitude) {fail(t, "relative altitude limit");}
      if (r.angular_acceleration_valid) {
        out.max_angular_acceleration = std::max(
          out.max_angular_acceleration,
          r.body_acceleration.norm());
        if (r.body_acceleration.norm() > l.angular_acceleration_max) {
          fail(t, "angular acceleration limit");
        }
        const Eigen::Vector3d torque = l.inertia.cwiseProduct(r.body_acceleration) +
          r.body_rate.cross(l.inertia.cwiseProduct(r.body_rate));
        Eigen::Vector4d wrench;wrench << l.mass * r.thrust_acceleration, torque;
        const Eigen::Vector4d motors = mix * wrench;
        out.min_motor = std::min(out.min_motor, motors.minCoeff());
        out.max_motor = std::max(out.max_motor, motors.maxCoeff());
        if (motors.minCoeff() < l.motor_min || motors.maxCoeff() > l.motor_max) {
          fail(t, "static motor allocation limit");
        }
      }
    } catch (const std::invalid_argument & e) {
      fail(t, e.what());
    }
  }
  return out;
}
}  // namespace px4ctrl
