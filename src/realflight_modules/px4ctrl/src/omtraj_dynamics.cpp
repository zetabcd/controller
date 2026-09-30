#include <px4ctrl/omtraj_dynamics.h>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace omtraj
{
Eigen::Matrix3d hat(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}
Eigen::Quaterniond exp(const Eigen::Vector3d & v)
{
  const double a = v.norm();
  if (a < 1e-10) {
    return Eigen::Quaterniond(1, v.x() / 2, v.y() / 2, v.z() / 2).normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, v / a));
}
Eigen::Vector3d log(const Eigen::Quaterniond & q)
{
  Eigen::AngleAxisd a(q.normalized());
  return a.angle() * a.axis();
}
Eigen::Vector4d input(const OmTrajectoryState & x)
{
  Eigen::Vector4d u;
  u << x.thrust_acceleration, x.body_rate;
  return u;
}
void setInput(OmTrajectoryState & x, const Eigen::Vector4d & u)
{
  x.thrust_acceleration = u(0);
  x.body_rate = u.tail<3>();
}
OmTrajectoryState retract(OmTrajectoryState x, const V9 & d)
{
  x.position += d.head<3>();
  x.velocity += d.segment<3>(3);
  x.attitude = (x.attitude * exp(
      d.tail<3>())).normalized();
  return x;
}
V9 difference(const OmTrajectoryState & x, const OmTrajectoryState & ref)
{
  V9 d;
  d << x.position - ref.position, x.velocity - ref.velocity, log(
    ref.attitude.conjugate() * x.attitude);
  return d;
}
Eigen::Vector3d acceleration(const OmTrajectoryState & x, const OmTrajectoryModel & m)
{
  const Eigen::Vector3d vb = x.attitude.conjugate() * x.velocity;
  Eigen::Vector3d f = -m.linear_drag.cwiseProduct(vb);
  f.z() += x.thrust_acceleration + m.horizontal_lift * vb.head<2>().squaredNorm();
  return x.attitude * f - m.gravity * Eigen::Vector3d::UnitZ();
}
OmTrajectoryState integrate(
  const OmTrajectoryState & x, const Eigen::Vector4d & u1,
  double dt, const OmTrajectoryModel & model, int substeps)
{
  if (dt == 0) {
    return x;
  }
  using V10 = Eigen::Matrix<double, 10, 1>;
  const Eigen::Vector4d u0 = input(x);
  V10 y;
  y << x.position, x.velocity, x.attitude.coeffs();
  const auto rhs = [&](const V10 & s, double fraction) {
      OmTrajectoryState r;
      r.position = s.head<3>();
      r.velocity = s.segment<3>(3);
      r.attitude = Eigen::Quaterniond(s.tail<4>()).normalized();
      setInput(r, (1 - fraction) * u0 + fraction * u1);
      Eigen::Quaterniond w(0, r.body_rate.x(), r.body_rate.y(), r.body_rate.z());
      V10 dy;
      dy << r.velocity, acceleration(r, model), 0.5 * (r.attitude * w).coeffs();
      return dy;
    };
  const double h = dt / substeps;
  for (int i = 0; i < substeps; ++i) {
    const double f = double(i) / substeps, df = 1.0 / substeps;
    const V10 k1 = rhs(y, f), k2 = rhs(y + h * k1 / 2, f + df / 2),
      k3 = rhs(y + h * k2 / 2, f + df / 2), k4 = rhs(y + h * k3, f + df);
    y += h * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    y.tail<4>().normalize();
  }
  OmTrajectoryState out = x;
  out.position = y.head<3>();
  out.velocity = y.segment<3>(3);
  out.attitude = Eigen::Quaterniond(y.tail<4>()).normalized();
  out.time = x.time + dt;
  setInput(out, u1);
  return out;
}
Eigen::Matrix4d allocationInverse(const OmTrackingLimits & l)
{
  const double a = l.arm * std::cos(l.arm_angle), b = l.arm * std::sin(l.arm_angle),
    k = l.torque_to_thrust;
  Eigen::Matrix4d B;
  B << 1, 1, 1, 1, b, -b, -b, b, -a, -a, a, a, k, -k, k, -k;
  if (std::abs(B.determinant()) < 1e-12) {
    throw std::invalid_argument("Singular motor allocation");
  }
  return B.inverse();
}
Eigen::VectorXd constraints(
  const OmTrajectoryState & x, const Eigen::Vector4d & slope,
  const OmTrajectoryOptions & o, double origin_z, ConstraintSample sample)
{
  const auto & l = o.tracking;
  std::vector<double> g;
  const auto box = [&](double v, double lo, double hi, double scale) {
      g.push_back((v - hi) / scale);
      g.push_back((lo - v) / scale);
    };
  const double tmin =
    l.enabled ? std::max(l.thrust_min, o.thrust_acceleration_min) : o.thrust_acceleration_min;
  const double tmax =
    l.enabled ? std::min(l.thrust_max, o.thrust_acceleration_max) : o.thrust_acceleration_max;
  const bool inputs = sample != ConstraintSample::Interior;
  const bool slew = sample == ConstraintSample::All;
  if (inputs) {
    box(x.thrust_acceleration, tmin, tmax, 10);
    for (int j = 0; j < 3; ++j) {
      const double limit =
        l.enabled ? std::min(l.rate_max(j), o.body_rate_max(j)) : o.body_rate_max(j);
      box(x.body_rate(j), -limit, limit, 1);
    }
  }
  if (l.enabled) {
    if (slew) {
      box(slope(0), -l.thrust_slew_max, l.thrust_slew_max, 10);
    }
    if (inputs) {
      box(x.thrust_acceleration + l.thrust_time_constant * slope(0), tmin, tmax, 10);
    }
    for (int j = 0; j < 3; ++j) {
      if (slew) {
        box(slope(j + 1), -l.angular_acceleration_max(j), l.angular_acceleration_max(j), 10);
      }

    }
    g.push_back((x.velocity.squaredNorm() - l.speed_max * l.speed_max) / (2 * l.speed_max));
    g.push_back(std::cos(l.maximum_tilt) - (x.attitude * Eigen::Vector3d::UnitZ()).z());
    g.push_back(origin_z + l.minimum_relative_altitude - x.position.z());
    if (l.motor_constraints) {
      Eigen::Vector4d wrench;
      wrench << l.mass * x.thrust_acceleration,
        l.inertia.cwiseProduct(slope.tail<3>()) +
        x.body_rate.cross(l.inertia.cwiseProduct(x.body_rate));
      const Eigen::Vector4d motors = allocationInverse(l) * wrench;
      for (int j = 0; j < 4; ++j) {
        box(motors(j), l.motor_min, l.motor_max, 1);
      }
    }
  }
  return Eigen::Map<Eigen::VectorXd>(g.data(), g.size());
}
}
