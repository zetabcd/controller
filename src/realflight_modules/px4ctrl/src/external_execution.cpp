#include <px4ctrl/external_execution.h>
#include <chrono>
#include <cmath>

namespace px4ctrl
{
namespace {
template<class V> Eigen::Vector3d vec(const V &v) {return {v.x,v.y,v.z};}
}
ExternalExecution::ExternalExecution(rclcpp::Node &n, TrajectoryLimits limits,
  ExternalModel model, bool simulation) : node_(n), limits_(limits), model_(model)
{
  if(n.declare_parameter("trajectory.external.simulation",simulation)!=simulation) {
    throw std::invalid_argument("gap_flight.launch.py SIMULATION differs from input.h PX4CTRL_SIMULATION; match both settings and rebuild px4ctrl after editing input.h");
  }
  takeoff_height_=n.get_parameter("trajectory.takeoff_height").as_double();
  if(!std::isfinite(takeoff_height_) || takeoff_height_<0) {throw std::invalid_argument("Invalid trajectory.takeoff_height");}
  takeoff_duration_=n.get_parameter("trajectory.takeoff_duration").as_double();
  settle_duration_=n.get_parameter("trajectory.settle_duration").as_double();
  if(!std::isfinite(takeoff_duration_) || takeoff_duration_<=0 ||
    !std::isfinite(settle_duration_) || settle_duration_<0) {
    throw std::invalid_argument("Invalid external preparation duration / settling time");
  }
  position_tolerance_=n.declare_parameter("trajectory.external.start_position_tolerance",0.10);
  speed_tolerance_=n.declare_parameter("trajectory.external.start_speed_tolerance",0.15);
  attitude_tolerance_=n.declare_parameter("trajectory.external.start_attitude_tolerance",0.15);
  scene_timeout_=n.declare_parameter("trajectory.external.scene_timeout",0.5);
  if(!(position_tolerance_>0 && speed_tolerance_>0 && attitude_tolerance_>0 && scene_timeout_>0)) {
    throw std::invalid_argument("Invalid external activation tolerances");
  }
  // No facility/world-box constraints in this mode. Gate collision and dynamic
  // checks are independent of any user-managed room dimensions.
  limits_.enforce_minimum_altitude=false;
  status_=n.create_publisher<gap_msgs::msg::ExecutionStatus>("/gap/execution",10);
  scene_=n.create_subscription<gap_msgs::msg::SceneStatus>("/gap/scene",10,
    [this](gap_msgs::msg::SceneStatus::ConstSharedPtr m) {
      const double stamp=rclcpp::Time(m->header.stamp).seconds();
      if(stamp<=scene_stamp_) {return;}
      scene_stamp_=stamp; scene_received_=node_.now().seconds(); latest_scene_=m->scene_id;
      scene_valid_=m->valid && m->header.frame_id=="world_nwu";
    });
  upload_=n.create_service<gap_msgs::srv::UploadTrajectory>("/gap/upload",
    [this](const std::shared_ptr<gap_msgs::srv::UploadTrajectory::Request> req,
      std::shared_ptr<gap_msgs::srv::UploadTrajectory::Response> res) {
      res->accepted=false;
      if(!command_mode_ || !hovering_ || !feedback_valid_ || active_ || validation_.valid()) {
        res->message=active_kind_==ActiveKind::Preparation ?
          "CMD is preparing: wait for target height and stable hold" :
          "Upload requires CMD holding and idle validator"; return;
      }
      if(req->samples.size()<2 || req->samples.size()>50000 || req->trajectory_id.empty() ||
        req->scene_id.empty() || req->header.frame_id!="world_nwu" || req->model.version!=1 ||
        !std::isfinite(req->model.mass) || !std::isfinite(req->model.gravity) ||
        !std::isfinite(req->model.heading) || !vec(req->model.drag_acceleration).allFinite() ||
        !std::isfinite(req->model.horizontal_lift_acceleration) ||
        std::abs(req->model.mass-model_.mass)>1e-6 ||
        std::abs(req->model.gravity-model_.gravity)>1e-6 ||
        (vec(req->model.drag_acceleration)-model_.drag).norm()>1e-6 ||
        std::abs(req->model.horizontal_lift_acceleration-model_.lift)>1e-6) {
        res->message="Invalid payload, frame or planner/controller model mismatch"; return;
      }
      pending_.reset(); start_requested_=false;
      trajectory_id_=req->trajectory_id; scene_id_=req->scene_id;
      state_="VALIDATING"; reason_="Reconstructing and auditing buffered reference";
      auto model=model_; model.heading=req->model.heading;
      const auto limits=limits_;const auto generation=generation_;
      validation_=std::async(std::launch::async,[req,model,limits,generation]() -> Validation {
        const auto began=std::chrono::steady_clock::now();
        const auto elapsed=[&]() {return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();};
        try {
          TimedReferences samples; samples.reserve(req->samples.size());
          for(const auto &s:req->samples) {
            ReferencePoint r; r.position=vec(s.position); r.velocity=vec(s.velocity);
            r.acceleration=vec(s.acceleration); samples.push_back({s.time,r});
          }
          auto t=std::make_shared<ExternalTrajectory>(std::move(samples),model);
          const auto audit=auditTrajectory(*t,limits,0.002);
          if(!audit.valid) {throw std::invalid_argument(audit.reason+" at "+std::to_string(audit.first_failure_time));}
          return {t,"",elapsed(),generation};
        } catch(const std::exception &e) {return {nullptr,e.what(),elapsed(),generation};}
      });
      res->accepted=true; res->message="Validation queued; await READY on /gap/execution";
    });
  // The original AUX2 path owns CMD entry in both modes. Each task has an
  // explicit planner-console start; no trajectory arrival auto-starts flight.
  start_=n.create_service<std_srvs::srv::Trigger>("/gap/start",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      res->success=ready(res->message);
      if(res->success) {start_requested_=true;res->message="Execution requested in CMD";}
    });
}

bool ExternalExecution::ready(std::string &why) const
{
  const double now=node_.now().seconds();
  if(active_) {why=active_kind_==ActiveKind::Preparation ?
    "CMD is preparing: wait for target height and stable hold" : "A trajectory is executing";
    return false;}
  if(!pending_) {why="No validated trajectory; stay in CMD holding"; return false;}
  if(!command_mode_ || !hovering_ || !feedback_valid_) {why="Requires valid CMD holding"; return false;}
  if(!scene_valid_ || latest_scene_!=scene_id_ || now-scene_received_<0 ||
    now-scene_received_>scene_timeout_ || now-scene_stamp_<-0.05 || now-scene_stamp_>scene_timeout_) {
    why="Scene moved, expired or unavailable; replan"; return false;
  }
  const auto first=pending_->evaluate(0);
  if((p_-first.position).norm()>position_tolerance_ || v_.norm()>speed_tolerance_ ||
    q_.angularDistance(first.attitude)>attitude_tolerance_) {
    why="Start state moved; replan from current hover"; return false;
  }
  why="READY; controller_validation_ms="+std::to_string(validation_ms_)+
    "; enter e to execute this leg"; return true;
}

void ExternalExecution::poll(double now, bool command_mode, bool auto_hover, bool valid,
  const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Quaterniond &q,
  bool origin_valid,const Eigen::Vector3d &origin)
{
  // beginCommand() owns CMD entry; never advertise WAITING before preparation.
  if(!command_mode && command_mode_) {stop();}
  command_mode_=command_mode;
  const bool hovering=(command_mode && !active_) || auto_hover;
  hovering_=hovering; feedback_valid_=valid; p_=p; v_=v; q_=q;
  if(last_now_>=0 && now<last_now_) {
    pending_.reset(); scene_valid_=false; scene_stamp_=-1; start_requested_=false;
    settled_since_=-1;++generation_;
    reason_="Clock reset; new scene and trajectory required"; state_="REJECTED";
    publish_at_=0;
  }
  last_now_=now;
  if(active_kind_==ActiveKind::Preparation && active_) {
    const auto end=active_->evaluate(active_->duration());
    const bool stable=now-started_>=active_->duration() && valid && p.allFinite() &&
      v.allFinite() && q.coeffs().allFinite() && q.norm()>1e-8 &&
      (p-end.position).norm()<=position_tolerance_ && v.norm()<=speed_tolerance_ &&
      q.normalized().angularDistance(end.attitude)<=attitude_tolerance_;
    if(!stable) {settled_since_=-1;}
    else if(settled_since_<0) {settled_since_=now;}
  }
  if(validation_.valid() && validation_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
    auto result=validation_.get();
    if(result.generation==generation_ && command_mode_) {
    pending_=result.trajectory;
    validation_ms_=result.elapsed_ms;
    state_=pending_?"READY":"REJECTED";
    reason_=result.reason+"; controller_validation_ms="+std::to_string(validation_ms_);
    RCLCPP_INFO(node_.get_logger(),"Trajectory %s: %s controller_validation_ms=%.2f",
      trajectory_id_.c_str(),state_.c_str(),validation_ms_);
    }
  }
  if(!command_mode && pending_) {pending_.reset();state_="OUTSIDE_CMD";reason_="Enter CMD to plan";}
  if(pending_) {
    if(ready(reason_)) {state_="READY";}
    else {pending_.reset(); state_="REJECTED"; start_requested_=false;}
  }
  if(active_kind_==ActiveKind::Uploaded && active_ && !hovering && (!scene_valid_ || latest_scene_!=scene_id_ ||
    now-scene_received_<0 || now-scene_received_>scene_timeout_ ||
    now-scene_stamp_<-0.05 || now-scene_stamp_>scene_timeout_)) {
    reason_="Scene changed or monitor lost during flight; executing immutable static-scene plan";
  }
  if(now<publish_at_) {return;} publish_at_=now+0.1;
  gap_msgs::msg::ExecutionStatus m; m.header.stamp=node_.now(); m.header.frame_id="world_nwu";
  m.state=state_; m.reason=reason_; m.trajectory_id=trajectory_id_;
  m.hovering=hovering;m.command_mode=command_mode;m.origin_valid=origin_valid;
  m.takeoff_position.x=origin.x();m.takeoff_position.y=origin.y();m.takeoff_position.z=origin.z();
  m.feedback_valid=valid;
  m.pose.position.x=p.x();m.pose.position.y=p.y();m.pose.position.z=p.z();
  m.pose.orientation.w=q.w();m.pose.orientation.x=q.x();m.pose.orientation.y=q.y();m.pose.orientation.z=q.z();
  m.velocity.x=v.x();m.velocity.y=v.y();m.velocity.z=v.z();m.elapsed=active_?now-started_:0;
  status_->publish(m);
}
bool ExternalExecution::requestStart()
{const bool requested=start_requested_; start_requested_=false; return requested;}

std::shared_ptr<const Trajectory> ExternalExecution::beginCommand(
  double now, const ReferencePoint &start)
{
  stop();
  if(!std::isfinite(now)) {
    throw std::invalid_argument("Invalid external preparation time");
  }
  Eigen::Vector3d target=start.position;target.z()+=takeoff_height_;
  auto preparation=std::make_shared<PointToPointTrajectory>(start,target,takeoff_duration_,model_.gravity);
  command_mode_=true;
  active_=std::move(preparation);active_kind_=ActiveKind::Preparation;
  started_=now;hovering_=false;state_="PREPARING";
  reason_="CMD moving to task height; planning/upload/start wait for stable hold";
  return active_;
}

std::shared_ptr<const Trajectory> ExternalExecution::activate(double now)
{
  std::string why;
  if(!ready(why)) {reason_=why;return nullptr;}
  active_=std::move(pending_);active_kind_=ActiveKind::Uploaded;hovering_=false;
  started_=now; state_="EXECUTING"; reason_="Playing buffered world trajectory";
  return active_;
}
bool ExternalExecution::finished(double now) const
{return active_ && now-started_>=active_->duration() &&
  (active_kind_!=ActiveKind::Preparation ||
   (settled_since_>=0 && now-settled_since_>=settle_duration_));}
void ExternalExecution::complete()
{
  const bool preparation=active_kind_==ActiveKind::Preparation;
  active_.reset();active_kind_=ActiveKind::None;settled_since_=-1;
  hovering_=command_mode_;
  state_=preparation ? "WAITING" : "COMPLETED";
  reason_="CMD holding: select a count or r to replan; AUX2 controls flight mode";
}
void ExternalExecution::stop()
{
  active_.reset();pending_.reset();active_kind_=ActiveKind::None;settled_since_=-1;
  start_requested_=false;command_mode_=false;hovering_=false;
  trajectory_id_.clear();scene_id_.clear();
  ++generation_;state_="OUTSIDE_CMD";reason_="Enter CMD to select a new task";
}
}
