#include <gap_planner/core.h>
#include <gap_planner/sparse_optimizer.h>
#include <chrono>
#include <iomanip>
#include <sstream>
namespace gap_planner {
namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point start) {return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
struct Measure {double &value;Clock::time_point begin{Clock::now()};~Measure() {value+=elapsed(begin);}};
class PolynomialReference final:public px4ctrl::Trajectory {
  const Result &result_;const Options &o_;
public:
  PolynomialReference(const Result &result,const Options &o):result_(result),o_(o) {}
  double duration() const override {return result_.duration;}
  std::vector<double> boundaries() const override {
    std::vector<double> out{0};for(const auto &p:result_.pieces) {out.push_back(out.back()+p.duration);}return out;
  }
  px4ctrl::ReferencePoint evaluate(double time) const override {
    for(size_t i=0;i<result_.pieces.size();++i) {
      const auto &p=result_.pieces[i];
      if(time<=p.duration || i+1==result_.pieces.size()) {return px4ctrl::externalReference(flatPoint(p,std::clamp(time,0.0,p.duration)),o_.model);}
      time-=p.duration;
    }
    throw std::runtime_error("Empty trajectory");
  }
};
std::string audit(Result &result,const std::vector<Gate> &gates,const FrameGeometry &frames,
  const Eigen::Vector3d &start,const Eigen::Vector3d &goal,const Options &o,std::vector<std::vector<double>> &refinement) {
  refinement.resize(result.pieces.size());
  result.min_clearance=INFINITY;result.endpoint_distance=(goal-start).norm();
  Eigen::Vector3d previous=start;double worst_time=0,max_tilt=0,min_thrust=INFINITY,max_thrust=0;
  std::string worst_frame;bool finite=true;
  for(size_t i=0;i<result.pieces.size();++i) {
    const auto &piece=result.pieces[i];
    if(!std::isfinite(piece.duration) || piece.duration<1e-4 || piece.duration>120 || !piece.coefficients.allFinite()) {return "nonfinite polynomial/duration";}
    std::vector<double> minima(frames.boxes.size(),INFINITY),at(frames.boxes.size(),0);
    const int steps=std::max(1,static_cast<int>(std::ceil(piece.duration/o.audit_dt)));
    for(int k=0;k<=steps;++k) {
      const double local=piece.duration*k/steps;
      const auto r=px4ctrl::externalReference(flatPoint(piece,local),o.model);
      const Eigen::Vector4d q(r.attitude.w(),r.attitude.x(),r.attitude.y(),r.attitude.z());
      finite=finite && r.position.allFinite() && r.velocity.allFinite() && r.body_rate.allFinite() && q.allFinite() && std::isfinite(r.thrust_acceleration);
      if(!finite) {return "nonfinite flatness reference";}
      result.path_length+=(r.position-previous).norm();
      result.backward_distance+=std::max(0.0,-(r.position-previous).dot((goal-start).normalized()));previous=r.position;
      for(size_t b=0;b<frames.boxes.size();++b) {
        const auto &box=frames.boxes[b];
        const double sphere_gap=(r.position-box.center).norm()-box.half.norm()-frames.radius;
        if(sphere_gap>std::min(minima[b],result.min_clearance)) {continue;}
        const double gap=frames.separation(box,r.position,q,false).distance;
        if(gap<minima[b]) {minima[b]=gap;at[b]=static_cast<double>(k)/steps;}
        if(gap<result.min_clearance) {result.min_clearance=gap;worst_frame=box.id;worst_time=result.duration+local;}
      }
      result.max_speed=std::max(result.max_speed,r.velocity.norm());result.max_rate=std::max(result.max_rate,r.body_rate.norm());
      max_tilt=std::max(max_tilt,std::acos(std::clamp(r.attitude.toRotationMatrix()(2,2),-1.0,1.0)));
      min_thrust=std::min(min_thrust,r.thrust_acceleration);max_thrust=std::max(max_thrust,r.thrust_acceleration);
    }
    for(size_t b=0;b<minima.size();++b) {if(minima[b]<o.margin+o.optimization_buffer) {refinement[i].push_back(at[b]);}}
    result.duration+=piece.duration;
    if(i%3==2 && i+1<result.pieces.size()) {
      const auto &gate=gates[i/3];const auto R=gate.rotation.toRotationMatrix();
      const auto r=px4ctrl::externalReference(flatPoint(piece,piece.duration),o.model);
      const Eigen::Vector3d local=gate.rotation.conjugate()*(r.position-gate.center);
      if(std::abs(local.x())>1e-6 || R.col(0).dot(r.velocity)<=0) {return "crossing direction at "+gate.id;}
      for(int axis:{1,2}) {for(int sign:{-1,1}) {
        const Eigen::Vector3d n=sign*R.col(axis);
        const double gap=(axis==1?gate.width:gate.height)/2-sign*local[axis]-bodySupport(r.attitude.conjugate()*n,o.body,o.vertices);
        if(gap<o.margin) {return "crossing aperture at "+gate.id+" clearance="+std::to_string(gap);}
      }}
      const auto rotation=r.attitude.toRotationMatrix();
      result.crossings.push_back({gate.id,result.duration,std::atan2(rotation(2,1),rotation(2,2))*180/M_PI,
        std::acos(std::clamp(rotation(2,2),-1.0,1.0))*180/M_PI});
    }
  }
  if(result.min_clearance<o.margin || result.max_speed>o.speed*1.001 || result.max_rate>o.rate*1.001 ||
    max_tilt>o.tilt+1e-4 || min_thrust<o.thrust_min*0.999 || max_thrust>o.thrust_max*1.001 || result.duration>120) {
    std::ostringstream reason;reason<<"dense audit: clearance="<<result.min_clearance<<" (required "<<o.margin
      <<", "<<worst_frame<<" t="<<worst_time<<") speed="<<result.max_speed<<" rate="<<result.max_rate
      <<" tilt="<<max_tilt<<" thrust=["<<min_thrust<<","<<max_thrust<<"]";return reason.str();
  }
  auto limits=o.execution_limits;limits.mass=o.model.mass;limits.gravity=o.model.gravity;limits.enforce_minimum_altitude=false;
  const auto execution=px4ctrl::auditTrajectory(PolynomialReference(result,o),limits,o.audit_dt);
  if(!execution.valid) {
    double time=execution.first_failure_time;
    for(size_t i=0;i<result.pieces.size();++i) {
      if(time<=result.pieces[i].duration || i+1==result.pieces.size()) {
        refinement[i].push_back(std::clamp(time/result.pieces[i].duration,0.0,1.0));break;
      }
      time-=result.pieces[i].duration;
    }
    std::ostringstream reason;reason<<"execution: "<<execution.reason<<" t="<<execution.first_failure_time
      <<" motors=["<<execution.min_motor<<","<<execution.max_motor<<"] angular_acceleration="<<execution.max_angular_acceleration;
    return reason.str();
  }
  return {};
}
Result planImpl(const std::vector<Gate> &all,size_t count,const Eigen::Vector3d &start,
  const Eigen::Vector3d &goal,const Options &o,PlanTiming &timing) {
  const auto begin=Clock::now();
  if(count==0 || count>all.size()) {throw std::invalid_argument("Invalid selected gate count");}
  if(!std::isfinite(o.model.mass) || o.model.mass<=0 || !std::isfinite(o.model.gravity) || o.model.gravity<=0 ||
    !o.model.drag.allFinite() || !std::isfinite(o.model.lift) || !std::isfinite(o.model.heading)) {throw std::invalid_argument("Invalid flight model");}
  const std::vector<Gate> gates(all.begin(),all.begin()+count);validateTask(gates,start,goal,o);
  for(double value:{o.speed,o.rate,o.thrust_min,o.thrust_max,o.tilt,o.time_weight,o.audit_dt,o.solve_budget}) {
    if(!std::isfinite(value) || value<=0) {throw std::invalid_argument("Nonfinite/nonpositive planning limit");}
  }
  if(o.thrust_max<=o.thrust_min || o.tilt>=M_PI || o.audit_dt<0.0001 || o.audit_dt>0.01) {throw std::invalid_argument("Invalid physical/audit limits");}
  const FrameGeometry frames(gates,o);
  for(const auto &endpoint:{start,goal}) {
    px4ctrl::ReferencePoint flat;flat.position=endpoint;const auto r=px4ctrl::externalReference(flat,o.model);
    const Eigen::Vector4d q(r.attitude.w(),r.attitude.x(),r.attitude.y(),r.attitude.z());
    for(const auto &box:frames.boxes) {
      if(frames.separation(box,endpoint,q,false).distance<o.margin) {throw std::invalid_argument("Hover endpoint intersects frame/margin: "+box.id);}
    }
  }
  SparseOptimizer solver(gates,start,goal,o);
  Eigen::VectorXd x=solver.initial;timing.variables=x.size();timing.pieces=3*(count+1);
  timing.setup_ms=elapsed(begin);std::string failure;
  // Usually one solve. Restore feasibility only when the independent dense
  // audit asks for it, retaining the same sparse MINCO state between attempts.
  for(int attempt=0;attempt<8;++attempt) {
    const double remaining=o.solve_budget-elapsed(begin)/1000;
    if(remaining<=0) {break;}
    timing.phase="optimize";const auto solve_begin=Clock::now();
    const int status=solver.solve(x,remaining/(attempt==0?3.0:1.0));
    const double ms=elapsed(solve_begin);timing.optimize_ms+=ms;
    timing.stage_ms.push_back(ms);timing.stage_status.push_back(status);
    timing.evaluations=solver.evaluations;timing.iterations=solver.iterations;
    Result result;result.pieces=solver.trajectory(x);timing.phase="audit";std::vector<std::vector<double>> refinement;
    {Measure measure{timing.audit_ms};failure=audit(result,gates,frames,start,goal,o,refinement);}
    if(failure.empty()) {return result;}
    timing.audit_details.push_back(failure);
    solver.refine(refinement);
    solver.weight=std::min(1e6,solver.weight*10);
  }
  throw std::runtime_error("No verified feasible trajectory: "+failure);
}
}
std::string timingText(const PlanTiming &t)
{
  std::ostringstream out;out<<std::fixed<<std::setprecision(2)
    <<"setup_ms="<<t.setup_ms<<" optimize_ms="<<t.optimize_ms
    <<" audit_ms="<<t.audit_ms<<" total_ms="<<t.total_ms<<" stages_ms=[";
  for(std::size_t i=0;i<t.stage_ms.size();++i) {if(i) {out<<",";}out<<t.stage_ms[i];}
  out<<"] lbfgs_status=[";
  for(std::size_t i=0;i<t.stage_status.size();++i) {if(i) {out<<",";}out<<t.stage_status[i];}
  out<<"] pieces="<<t.pieces<<" variables="<<t.variables<<" evaluations="<<t.evaluations<<" iterations="<<t.iterations<<" phase="<<t.phase;
  for(size_t i=0;i<t.audit_details.size();++i) {out<<" retry["<<i<<"]="<<t.audit_details[i];}return out.str();
}
Result plan(const std::vector<Gate> &gates,std::size_t count,const Eigen::Vector3d &start,
  const Eigen::Vector3d &goal,const Options &o)
{
  PlanTiming timing;const auto begin=Clock::now();
  try {
    auto result=planImpl(gates,count,start,goal,o,timing);
    timing.total_ms=elapsed(begin);timing.phase="complete";result.timing=timing;return result;
  } catch(const std::exception &e) {
    timing.total_ms=elapsed(begin);throw PlanError(e.what(),timing);
  }
}
}
