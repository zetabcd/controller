#pragma once
#include <px4ctrl/external_trajectory.h>
#include <gap_planner/body_geometry.h>
#include <unsupported/Eigen/AutoDiff>

namespace gap_planner
{
// Match the controller's nominal aerodynamic map through angular acceleration.
// Use the faster first-order/9-variable derivative when no actuator penalty is
// active; otherwise propagate snap through second-order jets and 12-variable AD.
class ProjectFlatness
{
  template<class S> using V=Eigen::Matrix<S,3,1>;
  template<class S> struct Jet {V<S> p,d;};
  template<class S> static Jet<S> normalize(const Jet<S> &x) {
    const S norm=x.p.norm();const V<S> p=x.p/norm;
    return {p,(x.d-p*p.dot(x.d))/norm};
  }
  template<class S> static Jet<S> cross(const Jet<S> &a,const Jet<S> &b) {
    return {a.p.cross(b.p),a.d.cross(b.p)+a.p.cross(b.d)};
  }
  template<class S> Eigen::Matrix<S,8,1> map(const V<S> &v,const V<S> &a,const V<S> &j) const {
    using M=Eigen::Matrix<S,3,3>;
    const V<S> gravity(0,0,model.gravity),heading(std::cos(model.heading),std::sin(model.heading),0);
    Jet<S> z=normalize<S>({a+gravity,j}),h{heading,V<S>::Zero()};
    Jet<S> y=normalize<S>(cross(z,h)),x=cross(y,z);
    M R,Rd;R<<x.p,y.p,z.p;Rd<<x.d,y.d,z.d;
    const V<S> vb=R.transpose()*v,vbd=Rd.transpose()*v+R.transpose()*a;
    V<S> aero=-model.drag.template cast<S>().cwiseProduct(vb);
    V<S> ad=-model.drag.template cast<S>().cwiseProduct(vbd);
    aero.z()+=model.lift*(vb.x()*vb.x()+vb.y()*vb.y());
    ad.z()+=2*model.lift*(vb.x()*vbd.x()+vb.y()*vbd.y());
    const V<S> force=a+gravity-R*aero,fd=j-Rd*aero-R*ad;
    z=normalize<S>({force,fd});h={R.col(0),Rd.col(0)};
    y=normalize<S>(cross(z,h));x=cross(y,z);
    R<<x.p,y.p,z.p;Rd<<x.d,y.d,z.d;
    const M W=R.transpose()*Rd;
    Eigen::Quaternion<S> q(R);q.normalize();
    Eigen::Matrix<S,8,1> result;
    result<<model.mass*force.norm(),q.w(),q.x(),q.y(),q.z(),
      (W(2,1)-W(1,2))*0.5,(W(0,2)-W(2,0))*0.5,(W(1,0)-W(0,1))*0.5;
    return result;
  }
  template<class S> struct SecondJet {V<S> p,d,dd;};
  template<class S> static SecondJet<S> normalized2(const SecondJet<S> &v) {
    const S n=v.p.norm(),nd=v.p.dot(v.d)/n,ndd=(v.d.squaredNorm()+v.p.dot(v.dd)-nd*nd)/n;
    const V<S> p=v.p/n,d=(v.d-p*nd)/n;
    return {p,d,(v.dd-p*ndd-2*d*nd)/n};
  }
  template<class S> static SecondJet<S> cross2(const SecondJet<S> &a,const SecondJet<S> &b) {
    return {a.p.cross(b.p),a.d.cross(b.p)+a.p.cross(b.d),a.dd.cross(b.p)+2*a.d.cross(b.d)+a.p.cross(b.dd)};
  }
  template<class S> Eigen::Matrix<S,11,1> mapFull(const V<S> &v,const V<S> &a,const V<S> &j,const V<S> &snap) const {
    using M=Eigen::Matrix<S,3,3>;
    const V<S> gravity(0,0,model.gravity),heading(std::cos(model.heading),std::sin(model.heading),0);
    SecondJet<S> z=normalized2<S>({a+gravity,j,snap}),h{heading,V<S>::Zero(),V<S>::Zero()};
    SecondJet<S> y=normalized2<S>(cross2(z,h)),x=cross2(y,z);
    M R,Rd,Rdd;R<<x.p,y.p,z.p;Rd<<x.d,y.d,z.d;Rdd<<x.dd,y.dd,z.dd;
    const V<S> vb=R.transpose()*v,vbd=Rd.transpose()*v+R.transpose()*a;
    const V<S> vbdd=Rdd.transpose()*v+2*Rd.transpose()*a+R.transpose()*j;
    V<S> aero=-model.drag.template cast<S>().cwiseProduct(vb),ad=-model.drag.template cast<S>().cwiseProduct(vbd),
      add=-model.drag.template cast<S>().cwiseProduct(vbdd);
    aero.z()+=model.lift*(vb.x()*vb.x()+vb.y()*vb.y());
    ad.z()+=2*model.lift*(vb.x()*vbd.x()+vb.y()*vbd.y());
    add.z()+=2*model.lift*(vbd.x()*vbd.x()+vbd.y()*vbd.y()+vb.x()*vbdd.x()+vb.y()*vbdd.y());
    const V<S> force=a+gravity-R*aero,fd=j-Rd*aero-R*ad,fdd=snap-Rdd*aero-2*Rd*ad-R*add;
    z=normalized2<S>({force,fd,fdd});h={R.col(0),Rd.col(0),Rdd.col(0)};
    y=normalized2<S>(cross2(z,h));x=cross2(y,z);
    R<<x.p,y.p,z.p;Rd<<x.d,y.d,z.d;Rdd<<x.dd,y.dd,z.dd;
    const M W=R.transpose()*Rd,A=R.transpose()*Rdd;
    Eigen::Quaternion<S> q(R);q.normalize();Eigen::Matrix<S,11,1> out;
    out<<model.mass*force.norm(),q.w(),q.x(),q.y(),q.z(),
      (W(2,1)-W(1,2))*0.5,(W(0,2)-W(2,0))*0.5,(W(1,0)-W(0,1))*0.5,
      (A(2,1)-A(1,2))*0.5,(A(0,2)-A(2,0))*0.5,(A(1,0)-A(0,1))*0.5;
    return out;
  }

public:
  px4ctrl::ExternalModel model;
  void reset(double mass,double gravity,double,double,double,double)
  {model.mass=mass;model.gravity=gravity;}
  void forward(const Eigen::Vector3d &v,const Eigen::Vector3d &a,const Eigen::Vector3d &j,
    double,double,double &thrust,Eigen::Vector4d &quat,Eigen::Vector3d &omega)
  {
    velocity_=v;acceleration_=a;jerk_=j;
    const auto out=map<double>(v,a,j);
    thrust=out[0];quat=out.segment<4>(1);omega=out.tail<3>();anchor_=quat;
  }
  void backward(const Eigen::Vector3d &gp,const Eigen::Vector3d &gv,double gt,
    const Eigen::Vector4d &gq,const Eigen::Vector3d &gw,Eigen::Vector3d &op,
    Eigen::Vector3d &ov,Eigen::Vector3d &oa,Eigen::Vector3d &oj,double &yaw,double &yaw_rate) const
  {
    op=gp;ov=gv;oa.setZero();oj.setZero();yaw=0;yaw_rate=0;
    if(gt==0 && gq.isZero() && gw.isZero()) {return;}
    using AD=Eigen::AutoDiffScalar<Eigen::Matrix<double,9,1>>;
    V<AD> v,a,j;
    for(int k=0;k<9;++k) {
      AD &x=k<3?v[k]:(k<6?a[k-3]:j[k-6]);
      x.value()=k<3?velocity_[k]:(k<6?acceleration_[k-3]:jerk_[k-6]);
      x.derivatives().setZero();x.derivatives()[k]=1;
    }
    const auto out=map<AD>(v,a,j);
    Eigen::Vector4d quaternion;for(int i=0;i<4;++i) {quaternion[i]=out[1+i].value();}
    const double sign=quaternion.dot(anchor_)<0?-1.0:1.0;
    Eigen::Matrix<double,9,1> gradient=gt*out[0].derivatives();
    for(int i=0;i<4;++i) {gradient+=sign*gq[i]*out[1+i].derivatives();}
    for(int i=0;i<3;++i) {gradient+=gw[i]*out[5+i].derivatives();}
    ov+=gradient.head<3>();oa=gradient.segment<3>(3);oj=gradient.tail<3>();
  }
  void forwardFull(const Eigen::Vector3d &v,const Eigen::Vector3d &a,const Eigen::Vector3d &j,const Eigen::Vector3d &s,
    double &thrust,Eigen::Vector4d &q,Eigen::Vector3d &omega,Eigen::Vector3d &alpha) {
    velocity_=v;acceleration_=a;jerk_=j;snap_=s;
    const auto out=mapFull<double>(v,a,j,s);thrust=out[0];q=out.segment<4>(1);anchor_=q;
    omega=out.segment<3>(5);alpha=out.tail<3>();
  }
  void backwardFull(const Eigen::Vector3d &gp,const Eigen::Vector3d &gv,double gt,const Eigen::Vector4d &gq,
    const Eigen::Vector3d &gw,const Eigen::Vector3d &galpha,Eigen::Vector3d &op,Eigen::Vector3d &ov,
    Eigen::Vector3d &oa,Eigen::Vector3d &oj,Eigen::Vector3d &os) const {
    os.setZero();
    if(galpha.isZero()) {double yaw,yr;backward(gp,gv,gt,gq,gw,op,ov,oa,oj,yaw,yr);return;}
    using AD=Eigen::AutoDiffScalar<Eigen::Matrix<double,12,1>>;V<AD> inputs[4];
    const Eigen::Vector3d values[4]={velocity_,acceleration_,jerk_,snap_};
    for(int k=0;k<12;++k) {
      auto &x=inputs[k/3][k%3];x.value()=values[k/3][k%3];x.derivatives().setZero();x.derivatives()[k]=1;
    }
    const auto out=mapFull<AD>(inputs[0],inputs[1],inputs[2],inputs[3]);
    Eigen::Vector4d q;for(int k=0;k<4;++k) {q[k]=out[1+k].value();}
    const double sign=q.dot(anchor_)<0?-1.0:1.0;
    Eigen::Matrix<double,12,1> gradient=gt*out[0].derivatives();
    for(int k=0;k<4;++k) {gradient+=sign*gq[k]*out[1+k].derivatives();}
    for(int k=0;k<3;++k) {gradient+=gw[k]*out[5+k].derivatives()+galpha[k]*out[8+k].derivatives();}
    op=gp;ov=gv+gradient.head<3>();oa=gradient.segment<3>(3);oj=gradient.segment<3>(6);os=gradient.tail<3>();
  }

private:
  Eigen::Vector3d velocity_,acceleration_,jerk_,snap_;
  Eigen::Vector4d anchor_;
};
}
