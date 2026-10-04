#include <px4ctrl/external_trajectory.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4ctrl
{
ReferencePoint externalReference(ReferencePoint r, const ExternalModel &m)
{
  if (!std::isfinite(m.heading) || !std::isfinite(m.mass) || m.mass <= 0) {
    throw std::invalid_argument("Invalid external trajectory model");
  }
  r = resolveGeometricReference(r, m.gravity,
    {std::cos(m.heading), std::sin(m.heading), 0}, HeadingAxis::BodyX);
  r = compensateReferenceAerodynamics(r, m.gravity, m.drag, m.lift);
  r.aerodynamics_included = true;
  r.model_linear_drag = m.drag;
  r.model_horizontal_lift = m.lift;
  // Preserve polynomial kinematics: the nominal one-step aero correction is
  // deliberately the same approximation as existing analytic references.
  return r;
}

ExternalTrajectory::ExternalTrajectory(TimedReferences p, ExternalModel m) : model_(m)
{
  if (p.size() < 2 || p.size() > 50000 || p.front().time != 0 ||
    !std::isfinite(p.back().time) || p.back().time > 120) {
    throw std::invalid_argument("External samples: require 2..50000, t0=0, duration<=120 s");
  }
  for (std::size_t i=0; i<p.size(); ++i) {
    const auto &r=p[i].point;
    if (!std::isfinite(p[i].time) || !r.position.allFinite() ||
      !r.velocity.allFinite() || !r.acceleration.allFinite() ||
      (i && (p[i].time-p[i-1].time < 1e-5 || p[i].time-p[i-1].time > 0.1))) {
      throw std::invalid_argument("Invalid external sample or interval outside [10 us,100 ms]");
    }
    times_.push_back(p[i].time);
  }
  if (p.front().point.velocity.norm()>1e-5 || p.back().point.velocity.norm()>1e-5 ||
    p.front().point.acceleration.norm()>1e-5 || p.back().point.acceleration.norm()>1e-5) {
    throw std::invalid_argument("External trajectory must start/end at stationary hover");
  }
  for (std::size_t i=0; i+1<p.size(); ++i) {
    const auto &a=p[i].point, &b=p[i+1].point;
    const double h=p[i+1].time-p[i].time;
    Polynomial c;
    c.col(0)=a.position; c.col(1)=h*a.velocity; c.col(2)=0.5*h*h*a.acceleration;
    const Eigen::Vector3d dp=b.position-c.col(0)-c.col(1)-c.col(2);
    const Eigen::Vector3d dv=h*b.velocity-c.col(1)-2*c.col(2);
    const Eigen::Vector3d da=h*h*b.acceleration-2*c.col(2);
    c.col(3)=10*dp-4*dv+0.5*da;
    c.col(4)=-15*dp+7*dv-da;
    c.col(5)=6*dp-3*dv+0.5*da;
    coefficients_.push_back(c);
  }
  start_.position=p.front().point.position; end_.position=p.back().point.position;
}

ReferencePoint ExternalTrajectory::evaluate(double t) const
{
  if (!std::isfinite(t)) {throw std::invalid_argument("Nonfinite external time");}
  if(t<0) {return externalReference(start_,model_);}
  if(t>=duration()) {return externalReference(end_,model_);}
  const auto it=std::upper_bound(times_.begin(),times_.end(),t);
  const std::size_t i=std::max<std::size_t>(1,it-times_.begin())-1;
  const double h=times_[i+1]-times_[i], u=(t-times_[i])/h;
  Eigen::Vector3d d[5];
  for(int k=0;k<5;++k) {
    d[k].setZero();
    for(int j=k;j<6;++j) {
      double f=1; for(int n=0;n<k;++n) {f*=j-n;}
      d[k]+=coefficients_[i].col(j)*(f*std::pow(u,j-k)/std::pow(h,k));
    }
  }
  ReferencePoint r;
  r.position=d[0]; r.velocity=d[1]; r.acceleration=d[2]; r.jerk=d[3]; r.snap=d[4];
  return externalReference(r,model_);
}
}
