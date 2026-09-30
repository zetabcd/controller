#include <px4ctrl/control_reference.h>

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace px4ctrl
{
namespace
{
struct Jet
{
  Eigen::Vector3d value, first, second;
};
Jet normalized(const Jet & v)
{
  const double n = v.value.norm();
  if (!std::isfinite(n) || n < 1e-6) {
    throw std::invalid_argument("Singular flat reference: thrust or heading");
  }
  const double nd = v.value.dot(v.first) / n;
  const double ndd = (v.first.squaredNorm() + v.value.dot(v.second) - nd * nd) / n;
  Jet out;
  out.value = v.value / n;
  out.first = (v.first - out.value * nd) / n;
  out.second = (v.second - out.value * ndd - 2 * out.first * nd) / n;
  return out;
}
Jet cross(const Jet & a, const Jet & b)
{
  return {a.value.cross(b.value), a.first.cross(b.value) + a.value.cross(b.first),
    a.second.cross(b.value) + 2 * a.first.cross(b.first) + a.value.cross(b.second)};
}
Eigen::Vector3d vee(const Eigen::Matrix3d & m)
{
  return 0.5 * Eigen::Vector3d(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), m(1, 0) - m(0, 1));
}
bool validQuaternion(const Eigen::Quaterniond & q)
{
  return q.coeffs().allFinite() && std::isfinite(q.norm()) && q.norm() > 1e-8;
}
}  // namespace

ReferencePoint resolveGeometricReference(
  ReferencePoint r, double gravity, const Eigen::Vector3d & heading, HeadingAxis axis)
{
  if (!std::isfinite(gravity) || gravity <= 0 || !heading.allFinite() ||
    !r.position.allFinite() || !r.velocity.allFinite() || !r.acceleration.allFinite() ||
    !r.jerk.allFinite() || !r.snap.allFinite())
  {
    throw std::invalid_argument("Invalid geometric reference");
  }
  const Jet force{r.acceleration + gravity * Eigen::Vector3d::UnitZ(), r.jerk, r.snap};
  const Jet z = normalized(force);
  const Jet auxiliary{heading, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  Jet x, y;
  if (axis == HeadingAxis::BodyX) {
    y = normalized(cross(z, auxiliary));
    x = cross(y, z);
  } else {
    x = normalized(cross(auxiliary, z));
    y = cross(z, x);
  }
  Eigen::Matrix3d rotation, first, second;
  rotation << x.value, y.value, z.value;
  first << x.first, y.first, z.first;
  second << x.second, y.second, z.second;
  r.attitude = Eigen::Quaterniond(rotation).normalized();
  r.thrust_acceleration = force.value.norm();
  r.body_rate = vee(rotation.transpose() * first);
  r.body_acceleration = vee(rotation.transpose() * second);
  r.yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  r.full_state = true;
  r.angular_acceleration_valid = true;
  r.kinematics_valid = true;
  return r;
}

ReferencePoint compensateReferenceAerodynamics(
  ReferencePoint r, double gravity, const Eigen::Vector3d & drag, double lift)
{
  if (!drag.allFinite() || (drag.array()<0).any() || !std::isfinite(lift) || lift<0) {
    throw std::invalid_argument("Invalid aerodynamic coefficients");
  }
  if (r.aerodynamics_included) {
    if ((r.model_linear_drag-drag).norm()>1e-6 ||
      std::abs(r.model_horizontal_lift-lift)>1e-6) {
      throw std::invalid_argument("Trajectory/controller aerodynamic model mismatch");
    }
    return r;
  }
  const Eigen::Matrix3d R=r.attitude.normalized().toRotationMatrix();
  r.thrust_rate_valid=false; // correcting force changes its time derivative
  const Eigen::Vector3d vb=R.transpose()*r.velocity;
  Eigen::Vector3d aero=-drag.cwiseProduct(vb);
  aero.z()+=lift*vb.head<2>().squaredNorm();
  const Eigen::Vector3d force=R*(Eigen::Vector3d(0,0,r.thrust_acceleration)-aero);
  if (force.norm()<1e-6) {throw std::invalid_argument("Singular aerodynamic reference");}
  if (!r.kinematics_valid || !r.angular_acceleration_valid) {
    // Preserve twist through inversion; Euler yaw reconstruction would jump at
    // the vertical circle's +/-90 degree pitch crossings.
    r.attitude=(Eigen::Quaterniond::FromTwoVectors(R.col(2),force.normalized())*r.attitude).normalized();
    r.thrust_acceleration=force.norm();r.angular_acceleration_valid=false;
    return r;
  }
  const auto hat=[](const Eigen::Vector3d &w) {
    Eigen::Matrix3d h;h<<0,-w.z(),w.y(),w.z(),0,-w.x(),-w.y(),w.x(),0;return h;
  };
  const Eigen::Matrix3d W=hat(r.body_rate);
  const Eigen::Matrix3d Rd=R*W, Rdd=R*(hat(r.body_acceleration)+W*W);
  const Eigen::Vector3d vbd=Rd.transpose()*r.velocity+R.transpose()*r.acceleration;
  const Eigen::Vector3d vbdd=Rdd.transpose()*r.velocity+
    2*Rd.transpose()*r.acceleration+R.transpose()*r.jerk;
  Eigen::Vector3d ad=-drag.cwiseProduct(vbd),add=-drag.cwiseProduct(vbdd);
  ad.z()+=2*lift*vb.head<2>().dot(vbd.head<2>());
  add.z()+=2*lift*(vbd.head<2>().squaredNorm()+vb.head<2>().dot(vbdd.head<2>()));
  const Jet f{r.acceleration+gravity*Eigen::Vector3d::UnitZ()-R*aero,
    r.jerk-Rd*aero-R*ad, r.snap-Rdd*aero-2*Rd*ad-R*add};
  const Jet z=normalized(f), heading{R.col(0),Rd.col(0),Rdd.col(0)};
  const Jet y=normalized(cross(z,heading)),x=cross(y,z);
  Eigen::Matrix3d rotation,first,second;
  rotation<<x.value,y.value,z.value;
  first<<x.first,y.first,z.first;
  second<<x.second,y.second,z.second;
  const auto original=r.attitude;
  r.attitude=Eigen::Quaterniond(rotation).normalized();
  if(original.dot(r.attitude)<0) {r.attitude.coeffs()*=-1;}
  r.thrust_acceleration=f.value.norm();
  r.body_rate=vee(rotation.transpose()*first);
  r.body_acceleration=vee(rotation.transpose()*second);
  return r;
}

ReferencePoint resolveReference(ReferencePoint r, double gravity)
{
  if (!std::isfinite(gravity) || gravity <= 0 ||
    !r.position.allFinite() || !r.velocity.allFinite())
  {
    throw std::invalid_argument("Invalid reference position, velocity or gravity");
  }
  if (r.full_state) {
    if ((r.kinematics_valid && (!r.acceleration.allFinite() || !r.jerk.allFinite() || !r.snap.allFinite())) ||
      !validQuaternion(r.attitude) || !std::isfinite(r.thrust_acceleration) ||
      r.thrust_acceleration <= 0 || !r.body_rate.allFinite() ||
      (r.angular_acceleration_valid && !r.body_acceleration.allFinite()) ||
      (r.thrust_rate_valid && !std::isfinite(r.thrust_rate)) ||
      (r.aerodynamics_included && (!r.model_linear_drag.allFinite() ||
      (r.model_linear_drag.array() < 0).any() || !std::isfinite(r.model_horizontal_lift) ||
      r.model_horizontal_lift < 0)))
    {
      throw std::invalid_argument("Invalid full-state reference");
    }
    r.attitude.normalize();
    r.acceleration = r.attitude * Eigen::Vector3d(0, 0, r.thrust_acceleration) -
      gravity * Eigen::Vector3d::UnitZ();
    if (r.aerodynamics_included) {
      if (!r.model_linear_drag.allFinite() || (r.model_linear_drag.array() < 0).any() ||
        !std::isfinite(r.model_horizontal_lift) || r.model_horizontal_lift < 0) {
        throw std::invalid_argument("Invalid reference aerodynamic model");
      }
      const Eigen::Vector3d vb = r.attitude.conjugate() * r.velocity;
      Eigen::Vector3d aero = -r.model_linear_drag.cwiseProduct(vb);
      aero.z() += r.model_horizontal_lift * vb.head<2>().squaredNorm();
      r.acceleration += r.attitude * aero;
    }
    const auto rotation = r.attitude.toRotationMatrix();
    r.yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    if (!r.angular_acceleration_valid) {r.body_acceleration.setZero();}
    return r;
  }
  if (!r.acceleration.allFinite() || !r.jerk.allFinite() || !r.snap.allFinite() ||
    !std::isfinite(r.yaw) || !std::isfinite(r.yaw_rate) || !std::isfinite(r.yaw_acceleration))
  {
    throw std::invalid_argument("Invalid flat reference derivatives");
  }
  const Jet force{r.acceleration + gravity * Eigen::Vector3d::UnitZ(), r.jerk, r.snap};
  const double c = std::cos(r.yaw), s = std::sin(r.yaw);
  const Eigen::Vector3d heading(-s, c, 0), tangent(-c, -s, 0);
  const Jet y_heading{heading, r.yaw_rate * tangent,
    r.yaw_acceleration * tangent - r.yaw_rate * r.yaw_rate * heading};
  const Jet z = normalized(force);
  // Euler yaw + thrust defines a local chart, not a global aerobatic attitude.
  // Full-state references carry their own quaternion and bypass this chart.
  if (z.value.z() <= 1e-6) {
    throw std::invalid_argument("Flat yaw reference requires upward thrust; use full-state for aerobatics");
  }
  const Jet x = normalized(cross(y_heading, z));
  const Jet y = cross(z, x);
  Eigen::Matrix3d rotation, first, second;
  rotation << x.value, y.value, z.value;
  first << x.first, y.first, z.first;
  second << x.second, y.second, z.second;
  r.attitude = Eigen::Quaterniond(rotation).normalized();
  r.thrust_acceleration = force.value.norm();
  r.body_rate = vee(rotation.transpose() * first);
  r.body_acceleration = vee(first.transpose() * first + rotation.transpose() * second);
  r.full_state = true;
  r.angular_acceleration_valid = true;
  r.kinematics_valid = true;
  return r;
}

ReferenceWindow extrapolateFlatReference(
  const ReferencePoint & r, double stamp, int horizon, double dt, double gravity)
{
  if (r.full_state || horizon < 0 || !std::isfinite(stamp) || !std::isfinite(dt) || dt <= 0) {
    throw std::invalid_argument("Invalid flat reference window request");
  }
  ReferenceWindow window{stamp, dt, {}};
  window.points.reserve(horizon + 1);
  for (int k = 0; k <= horizon; ++k) {
    const double t = k * dt, t2 = t * t, t3 = t2 * t, t4 = t3 * t;
    auto p = r;
    p.position += t * r.velocity + t2 / 2 * r.acceleration + t3 / 6 * r.jerk + t4 / 24 * r.snap;
    p.velocity += t * r.acceleration + t2 / 2 * r.jerk + t3 / 6 * r.snap;
    p.acceleration += t * r.jerk + t2 / 2 * r.snap;
    p.jerk += t * r.snap;
    p.yaw += t * r.yaw_rate + t2 / 2 * r.yaw_acceleration;
    p.yaw_rate += t * r.yaw_acceleration;
    window.points.push_back(resolveReference(p, gravity));
  }
  return window;
}

void validateReferenceWindow(const ReferenceWindow & w, int horizon, double dt)
{
  if (horizon < 0 || !std::isfinite(w.stamp) || !std::isfinite(w.dt) || w.dt <= 0 ||
    !std::isfinite(dt) || dt <= 0 || std::abs(w.dt - dt) > 1e-9 ||
    w.points.size() != static_cast<std::size_t>(horizon + 1))
  {
    throw std::invalid_argument("Reference window size/time grid mismatch");
  }
  for (const auto & r : w.points) {
    if ((r.kinematics_valid && (!r.jerk.allFinite() || !r.snap.allFinite())) ||
      !r.full_state || !r.position.allFinite() || !r.velocity.allFinite() ||
      !r.acceleration.allFinite() || !std::isfinite(r.yaw) ||
      !validQuaternion(r.attitude) || std::abs(r.attitude.norm() - 1) > 1e-6 ||
      !std::isfinite(r.thrust_acceleration) || r.thrust_acceleration <= 0 ||
      !r.body_rate.allFinite() || (r.angular_acceleration_valid && !r.body_acceleration.allFinite()) ||
      (r.thrust_rate_valid && !std::isfinite(r.thrust_rate)) ||
      (r.aerodynamics_included && (!r.model_linear_drag.allFinite() ||
      (r.model_linear_drag.array() < 0).any() || !std::isfinite(r.model_horizontal_lift) ||
      r.model_horizontal_lift < 0)))
    {
      throw std::invalid_argument("Invalid resolved control reference");
    }
  }
}

Eigen::Vector3d angularFeedforward(const ReferencePoint & r, const Eigen::Quaterniond & actual)
{
  if (!r.angular_acceleration_valid) {return Eigen::Vector3d::Zero();}
  if (!validQuaternion(actual)) {throw std::invalid_argument("Invalid feedback attitude");}
  return actual.normalized().conjugate() * (r.attitude * r.body_acceleration);
}

void differentiateAngularRate(ReferenceWindow & w)
{
  if (w.points.size() < 2 || !std::isfinite(w.dt) || w.dt <= 0) {
    throw std::invalid_argument("Angular reconstruction requires at least two timed points");
  }
  for (std::size_t k = 0; k < w.points.size(); ++k) {
    const auto a = k == 0 ? 0 : k - 1;
    const auto b = std::min(k + 1, w.points.size() - 1);
    const auto & ra = w.points[a];
    const auto & rb = w.points[b];
    auto & r = w.points[k];
    r.body_acceleration = r.attitude.conjugate() *
      ((rb.attitude * rb.body_rate - ra.attitude * ra.body_rate) / ((b - a) * w.dt));
    r.angular_acceleration_valid = true;
  }
}
}  // namespace px4ctrl
