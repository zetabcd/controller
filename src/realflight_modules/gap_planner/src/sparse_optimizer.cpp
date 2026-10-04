#include <gap_planner/sparse_optimizer.h>
namespace gap_planner {
namespace {
double positive(double x) {return x>0?1+x+0.5*x*x:1/(1-x+0.5*x*x);}
double inverse(double t) {return t>1?std::sqrt(2*t-1)-1:1-std::sqrt(2/t-1);}
double derivative(double x) {return x>0?1+x:(1-x)*std::pow(positive(x),2);}
void basis(double t,Eigen::Matrix<double,6,1> (&b)[5]) {
  for(int k=0;k<5;++k) {b[k].setZero();for(int j=k;j<6;++j) {
    double v=1;for(int m=0;m<k;++m) {v*=j-m;}b[k][j]=v*std::pow(t,j-k);
  }}
}
}
SparseOptimizer::SparseOptimizer(const std::vector<Gate> &gates,const Eigen::Vector3d &start,
  const Eigen::Vector3d &goal,const Options &o):gates_(gates),o_(o),frames_(gates,o) {
  const auto &l=o.execution_limits;const double a=l.arm*std::cos(l.arm_angle),b=l.arm*std::sin(l.arm_angle),k=l.torque_to_thrust;
  Eigen::Matrix4d mixing;mixing<<1,1,1,1,b,-b,-b,b,-a,-a,a,a,k,-k,k,-k;allocation_=mixing.inverse();
  flat_.model=o.model;energy_scale=1/(o.model.gravity*o.model.gravity);
  pieces_=3*(gates.size()+1);variables_=pieces_+6*(gates.size()+1)+2*gates.size();
  samples_.resize(pieces_);times_.resize(pieces_);points_.resize(3,pieces_-1);initial=Eigen::VectorXd::Zero(variables_);
  Eigen::Matrix3d head=Eigen::Matrix3d::Zero(),tail=head;head.col(0)=start;tail.col(0)=goal;
  minco_.setConditions(head,tail,pieces_);
  Eigen::Vector3d previous=start;int offset=pieces_,piece=0;
  for(size_t leg=0;leg<=gates.size();++leg) {
    const Eigen::Vector3d end=leg==gates.size()?goal:gates[leg].center;
    const Eigen::Vector3d chord=end-previous;
    const Eigen::Vector3d direction=chord.norm()>1e-6?chord.normalized().eval():Eigen::Vector3d::UnitX().eval();
    const Eigen::Vector3d from=leg==0?direction:(gates[leg-1].rotation*Eigen::Vector3d::UnitX()).eval();
    const Eigen::Vector3d to=leg==gates.size()?direction:(gates[leg].rotation*Eigen::Vector3d::UnitX()).eval();
    const double offset_distance=std::clamp(chord.norm()/3,0.2,0.6);
    const Eigen::Vector3d p1=previous+offset_distance*from,p2=end-offset_distance*to;
    const double duration=std::max(0.8,(2*offset_distance+(p2-p1).norm())/(0.65*o.speed));
    for(int k=1;k<=3;++k) {
      initial[piece++]=inverse(duration/3);
      if(k<3) {initial.segment<3>(offset)=k==1?p1:p2;offset+=3;}
      else if(leg<gates.size()) {offset+=2;}
    }
    previous=end;
  }
}
void SparseOptimizer::decode(const Eigen::VectorXd &x) {
  for(int i=0;i<pieces_;++i) {times_[i]=positive(x[i]);}
  int offset=pieces_;
  for(int i=0;i<pieces_-1;++i) {
    if(i%3==2) {
      const auto &gate=gates_[i/3];const auto R=gate.rotation.toRotationMatrix();
      points_.col(i)=gate.center+R.col(1)*x[offset]+R.col(2)*x[offset+1];offset+=2;
    } else {points_.col(i)=x.segment<3>(offset);offset+=3;}
  }
}
double SparseOptimizer::sample(const Eigen::Vector3d &p,const Eigen::Vector3d &v,
  const Eigen::Vector3d &a,const Eigen::Vector3d &j,const Eigen::Vector3d &snap,int gate_index,Eigen::Vector3d &gp,
  Eigen::Vector3d &gv,Eigen::Vector3d &ga,Eigen::Vector3d &gj,Eigen::Vector3d &gs) {
  double thrust;Eigen::Vector4d q;Eigen::Vector3d omega,alpha,galpha=Eigen::Vector3d::Zero();
  if(gate_index<0) {flat_.forwardFull(v,a,j,snap,thrust,q,omega,alpha);}
  else {flat_.forward(v,a,j,0,0,thrust,q,omega);}
  gp.setZero();gv.setZero();Eigen::Vector4d gq=Eigen::Vector4d::Zero();
  Eigen::Vector3d gw=Eigen::Vector3d::Zero();double gt=0,cost=0;
  // Dimensionless smoothed hinge costs, with a small physical reserve. Avoid
  // applying the same 1e7 multiplier to metres, squared speeds and thrusts.
  const auto hinge=[&](double violation) {
    if(violation<=0) {return 0.0;}
    constexpr double smoothing=0.01;
    if(violation<smoothing) {cost+=weight*violation*violation/(2*smoothing);return weight*violation/smoothing;}
    cost+=weight*(violation-0.5*smoothing);return weight;
  };
  if(gate_index<0) {
    const auto &l=o_.execution_limits;
    const Eigen::Vector3d torque=l.inertia.cwiseProduct(alpha)+omega.cross(l.inertia.cwiseProduct(omega));
    Eigen::Vector4d wrench;wrench<<thrust,torque;const Eigen::Vector4d motors=allocation_*wrench;
    const double motor_scale=o_.model.mass*o_.model.gravity/4;
    Eigen::Vector4d gm;
    for(int k=0;k<4;++k) {
      gm[k]=(hinge((motors[k]-l.motor_max+0.02*motor_scale)/motor_scale)-
        hinge((l.motor_min+0.02*motor_scale-motors[k])/motor_scale))/motor_scale;
    }
    const Eigen::Vector4d gradient=allocation_.transpose()*gm;gt+=gradient[0];
    const Eigen::Vector3d torque_gradient=gradient.tail<3>();
    galpha+=l.inertia.cwiseProduct(torque_gradient);
    gw+=(l.inertia.cwiseProduct(omega)).cross(torque_gradient)+l.inertia.cwiseProduct(torque_gradient.cross(omega));
    for(int k=0;k<3;++k) {
      const double bound=l.rate_max[k]*0.995;
      gw[k]+=hinge(omega[k]*omega[k]/(bound*bound)-1)*2*omega[k]/(bound*bound);
    }
    const double alpha_max=l.angular_acceleration_max*0.995;
    galpha+=hinge(alpha.squaredNorm()/(alpha_max*alpha_max)-1)*2*alpha/(alpha_max*alpha_max);
    const double reserve=o_.margin+o_.optimization_buffer;
    for(const auto &box:frames_.boxes) {
      if((p-box.center).norm()>box.half.norm()+frames_.radius+reserve) {continue;}
      const auto s=frames_.separation(box,p,q);
      const double d=hinge((reserve-s.distance)/0.1)/0.1;
      gp-=d*s.position_gradient;gq-=d*s.quaternion_gradient;
    }
    const double speed=o_.speed*0.995,rate=o_.rate*0.995;
    gv+=hinge(v.squaredNorm()/(speed*speed)-1)*2*v/(speed*speed);
    gw+=hinge(omega.squaredNorm()/(rate*rate)-1)*2*omega/(rate*rate);
    const double cos_tilt=1-2*(q[1]*q[1]+q[2]*q[2]);
    gq+=hinge(std::cos(o_.tilt-0.002)-cos_tilt)*Eigen::Vector4d(0,4*q[1],4*q[2],0);
    const double acceleration=thrust/o_.model.mass;
    gt+=(hinge((acceleration-o_.thrust_max*0.995)/o_.model.gravity)-
      hinge((o_.thrust_min*1.005-acceleration)/o_.model.gravity))/(o_.model.mass*o_.model.gravity);
  } else {
    const auto &gate=gates_[gate_index];const auto R=gate.rotation.toRotationMatrix();
    const Eigen::Quaterniond rotation(q[0],q[1],q[2],q[3]);
    // At the explicit crossing knot, the convex body projection must fit the
    // opening. Away from that instant only the finite frame is an obstacle.
    for(int axis:{1,2}) {for(int sign:{-1,1}) {
      const Eigen::Vector3d n=sign*R.col(axis),local=rotation.conjugate()*n;
      Eigen::Vector3d vertex;double support;
      if(o_.vertices.empty()) {
        support=o_.body.cwiseProduct(local).norm();vertex=o_.body.cwiseAbs2().cwiseProduct(local)/support;
      } else {
        support=-INFINITY;for(const auto &point:o_.vertices) {if(local.dot(point)>support) {support=local.dot(point);vertex=point;}}
      }
      const double half=(axis==1?gate.width:gate.height)/2;
      const double d=hinge((n.dot(p-gate.center)+support+o_.margin+o_.optimization_buffer-half)/0.1)/0.1;
      gp+=d*n;gq+=d*rotatedVertexGradient(q,vertex,n);
    }}
    // Positive traversal at the assigned knot, not a global X monotonicity cost.
    gv-=hinge((0.05-R.col(0).dot(v))/o_.speed)*R.col(0)/o_.speed;
  }
  Eigen::Vector3d op,ov;
  flat_.backwardFull(gp,gv,gt,gq,gw,galpha,op,ov,ga,gj,gs);gp=op;gv=ov;
  return cost;
}
double SparseOptimizer::evaluate(const Eigen::VectorXd &x,Eigen::VectorXd &gradient) {
  ++evaluations;decode(x);gradient=Eigen::VectorXd::Zero(variables_);
  if(!times_.allFinite() || times_.minCoeff()<1e-4 || times_.maxCoeff()>120 || !points_.allFinite()) {return 1e100;}
  minco_.setParameters(points_,times_);double cost;minco_.getEnergy(cost);
  Eigen::MatrixX3d gc;Eigen::VectorXd gt;
  minco_.getEnergyPartialGradByCoeffs(gc);minco_.getEnergyPartialGradByTimes(gt);
  cost*=energy_scale;gc*=energy_scale;gt*=energy_scale;
  for(int i=0;i<pieces_;++i) {
    const Eigen::Matrix<double,6,3> c=minco_.getCoeffs().block<6,3>(6*i,0);
    for(int k=0;k<=resolution+1+static_cast<int>(samples_[i].size());++k) {
      const bool crossing=k==resolution+1,extra=k>resolution+1;
      if(crossing && (i%3!=2 || i==pieces_-1)) {continue;}
      const double alpha=extra?samples_[i][k-resolution-2]:(crossing?1.0:static_cast<double>(k)/resolution);
      const double quadrature=(k==0 || k==resolution)?0.5:1;
      const double factor=(crossing || extra)?1.0:quadrature*times_[i]/resolution;
      Eigen::Matrix<double,6,1> b[5];basis(alpha*times_[i],b);
      const Eigen::Vector3d p=c.transpose()*b[0],v=c.transpose()*b[1],a=c.transpose()*b[2],j=c.transpose()*b[3],s=c.transpose()*b[4];
      Eigen::Vector3d gp,gv,ga,gj,gs;
      const double penalty=sample(p,v,a,j,s,crossing?i/3:-1,gp,gv,ga,gj,gs);
      gc.block<6,3>(6*i,0)+=factor*(b[0]*gp.transpose()+b[1]*gv.transpose()+b[2]*ga.transpose()+b[3]*gj.transpose()+b[4]*gs.transpose());
      gt[i]+=factor*alpha*(gp.dot(v)+gv.dot(a)+ga.dot(j)+gj.dot(s)+gs.dot(120*c.row(5).transpose()));
      if(!crossing && !extra) {gt[i]+=quadrature*penalty/resolution;}
      cost+=factor*penalty;
    }
  }
  Eigen::Matrix3Xd gpoints;Eigen::VectorXd gtimes;
  minco_.propogateGrad(gc,gt,gpoints,gtimes);
  cost+=o_.time_weight*times_.sum();gtimes.array()+=o_.time_weight;
  for(int i=0;i<pieces_;++i) {gradient[i]=gtimes[i]*derivative(x[i]);}
  int offset=pieces_;
  for(int i=0;i<pieces_-1;++i) {
    if(i%3==2) {
      const auto R=gates_[i/3].rotation.toRotationMatrix();
      gradient[offset++]=R.col(1).dot(gpoints.col(i));gradient[offset++]=R.col(2).dot(gpoints.col(i));
    } else {gradient.segment<3>(offset)=gpoints.col(i);offset+=3;}
  }
  if(!std::isfinite(cost) || !gradient.allFinite()) {gradient.setZero();return 1e100;}
  return cost;
}
void SparseOptimizer::refine(const std::vector<std::vector<double>> &samples) {
  for(int i=0;i<pieces_;++i) {for(double alpha:samples[i]) {
    for(int k=-10;k<=10;++k) {
      const double value=std::clamp(alpha+k*0.001/times_[i],0.0,1.0);
      bool duplicate=false;for(double old:samples_[i]) {if(std::abs(value-old)<1e-4) {duplicate=true;break;}}
      if(!duplicate) {samples_[i].push_back(value);}
    }
  }}
}
int SparseOptimizer::solve(Eigen::VectorXd &x,double seconds) {
  deadline_=std::chrono::steady_clock::now()+std::chrono::milliseconds(static_cast<int>(seconds*1000));
  lbfgs::lbfgs_parameter_t params;params.mem_size=32;params.max_iterations=500;params.past=5;
  params.delta=1e-5;params.g_epsilon=1e-6;params.min_step=1e-20;
  double cost;
  auto progress=[](void *ptr,const Eigen::VectorXd &,const Eigen::VectorXd &,double,double,int k,int)->int {
    auto &self=*static_cast<SparseOptimizer *>(ptr);self.iterations+=1;
    (void)k;return std::chrono::steady_clock::now()>self.deadline_;
  };
  return lbfgs::lbfgs_optimize(x,cost,&SparseOptimizer::callback,nullptr,progress,this,params);
}
std::vector<Piece> SparseOptimizer::trajectory(const Eigen::VectorXd &x) {
  decode(x);minco_.setParameters(points_,times_);std::vector<Piece> out;
  for(int i=0;i<pieces_;++i) {out.push_back({times_[i],minco_.getCoeffs().block<6,3>(6*i,0).transpose()});}
  return out;
}
}
