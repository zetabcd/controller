#include <gap_planner/ros_conversion.h>
#include <gap_planner/mocap_input.h>
#include <gap_msgs/msg/polynomial_plan.hpp>
#include <gap_msgs/msg/scene_status.hpp>
#include <gap_msgs/msg/execution_status.hpp>
#include <gap_msgs/srv/plan_gaps.hpp>
#include <gap_msgs/srv/configure_gates.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include <chrono>
#include <memory>

using namespace gap_planner;
class GapPlannerNode : public rclcpp::Node
{
  struct TrackedGate {
    Gate gate;
    Eigen::Vector3d offset;
    Eigen::Quaterniond offset_rotation;
    double stamp{-1},received{-1};
    std::string last_rejection;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription;
  };
  Options options_;
  std::vector<TrackedGate> tracked_;
  std::vector<Gate> snapshot_;
  std::string scene_id_,trajectory_id_,mocap_frame_;
  bool simulation_,scene_valid_{false};
  double pose_timeout_,move_tolerance_,angle_tolerance_,goal_offset_x_,flight_height_;
  Eigen::Vector3d task_start_,task_goal_;
  std::vector<Gate> task_gates_;
  std::size_t selected_count_{0};
  bool returning_{false},configuring_{false};
  rclcpp::Client<gap_msgs::srv::ConfigureGates>::SharedPtr configure_;
  std::chrono::steady_clock::time_point configure_started_;
  Eigen::Vector3d world_translation_;
  Eigen::Quaterniond world_rotation_;
  std::unique_ptr<MocapInput> mocap_input_;
  gap_msgs::msg::ExecutionStatus state_;
  double state_received_{-1};
  std::future<Result> worker_;
  std::size_t requested_count_{0};
  double last_tick_{-1}, last_visual_{-1};
  std::shared_ptr<Result> displayed_plan_;
  std::vector<std::pair<std::size_t,std::size_t>> body_edges_;
  rclcpp::Publisher<gap_msgs::msg::PolynomialPlan>::SharedPtr plans_;
  rclcpp::Publisher<gap_msgs::msg::SceneStatus>::SharedPtr scenes_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<gap_msgs::msg::ExecutionStatus>::SharedPtr state_sub_;
  rclcpp::Service<gap_msgs::srv::PlanGaps>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector3d vparam(const std::string &name,std::vector<double> value)
  {
    const auto v=declare_parameter(name,value);
    if(v.size()!=3) {throw std::invalid_argument(name+" requires 3 values");}
    Eigen::Vector3d r(v[0],v[1],v[2]);
    if(!r.allFinite()) {throw std::invalid_argument(name+" must be finite");}return r;
  }
  Eigen::Quaterniond qparam(const std::string &name)
  {
    const auto v=declare_parameter(name,std::vector<double>{1,0,0,0});
    if(v.size()!=4) {throw std::invalid_argument(name+" requires w,x,y,z");}
    Eigen::Quaterniond q(v[0],v[1],v[2],v[3]);
    if(!q.coeffs().allFinite() || q.norm()<1e-8) {throw std::invalid_argument(name+" invalid quaternion");}
    return q.normalized();
  }
  void report(const std::string &s)
  {std_msgs::msg::String m;m.data=s;status_->publish(m);RCLCPP_INFO(get_logger(),"%s",s.c_str());}
  bool fresh(std::string &why)
  {
    if(simulation_) {return true;}
    const double now=get_clock()->now().seconds();
    for(std::size_t i=0;i<selected_count_;++i) {
      const auto &t=tracked_[i];
      if(t.received<0 || now-t.received>pose_timeout_ || now-t.stamp>pose_timeout_ || now-t.stamp < -0.05) {
        why="Missing/stale rigid body: "+t.gate.id+
          (t.last_rejection.empty()?"":"; last rejected message: "+t.last_rejection);return false;
      }
    }
    return true;
  }
  bool snapshotMatches(std::string &why)
  {
    if(snapshot_.size()!=selected_count_ || !fresh(why)) {return false;}
    for(std::size_t i=0;i<selected_count_;++i) {
      if((snapshot_[i].center-tracked_[i].gate.center).norm()>move_tolerance_ ||
        snapshot_[i].rotation.angularDistance(tracked_[i].gate.rotation)>angle_tolerance_) {
        why="Gate moved: "+tracked_[i].gate.id;return false;
      }
    }
    return true;
  }
  void launchPlan()
  {
    if(!state_.command_mode || !state_.hovering || !state_.feedback_valid) {
      scene_valid_=false;report("FAILED: controller left CMD holding during scene selection");return;
    }
    const auto gates=task_gates_;const auto start=task_start_,goal=task_goal_;const auto opt=options_;
    worker_=std::async(std::launch::async,[gates,start,goal,opt]() {return plan(gates,gates.size(),start,goal,opt);});
    report(std::string("PLANNING ")+(returning_?"RETURN":"OUTBOUND")+" count="+std::to_string(gates.size())+
      " goal=["+std::to_string(goal.x())+","+std::to_string(goal.y())+","+std::to_string(goal.z())+
      "]; CMD holds position; wait for READY");
  }
  void request(const std::shared_ptr<gap_msgs::srv::PlanGaps::Request> req,
    std::shared_ptr<gap_msgs::srv::PlanGaps::Response> res)
  {
    res->accepted=false;std::string why;const double now=this->now().seconds();
    if(worker_.valid() || configuring_) {res->message="Planner/scene selection is busy";return;}
    if(req->count==0 || req->count>tracked_.size()) {res->message="Count must be 1..gate_order.size";return;}
    if(now-state_received_>0.5 || state_received_<0 || !state_.feedback_valid || !state_.origin_valid) {
      res->message="Waiting for fresh controller feedback and takeoff origin";return;
    }
    if(!state_.command_mode) {res->message="Enter CMD before choosing count";return;}
    if(!state_.hovering) {res->message="Wait for the current flight to finish in CMD";return;}
    const double holding_speed=vector(state_.velocity).norm();
    if(!std::isfinite(holding_speed) || holding_speed>0.12) {
      res->message="CMD is settling: measured speed="+std::to_string(holding_speed)+
        " m/s; choose count again once below 0.12 m/s";return;
    }
    const auto old_count=selected_count_;selected_count_=req->count;
    if(!fresh(why)) {selected_count_=old_count;res->message=why;return;}
    if(simulation_ && !configure_->service_is_ready()) {
      selected_count_=old_count;res->message="Simulation gate-selection service is unavailable";return;
    }
    const Eigen::Vector3d home=vector(state_.takeoff_position)+Eigen::Vector3d(0,0,flight_height_);
    const Eigen::Vector3d far=home+Eigen::Vector3d(goal_offset_x_,0,0);
    task_start_=vector(state_.pose.position);
    returning_=(task_start_-far).norm()<(task_start_-home).norm();
    task_goal_=returning_?home:far;
    displayed_plan_.reset();snapshot_.clear();
    for(std::size_t i=0;i<selected_count_;++i) {snapshot_.push_back(tracked_[i].gate);}
    task_gates_=snapshot_;
    if(returning_) {
      std::reverse(task_gates_.begin(),task_gates_.end());
      for(auto &g:task_gates_) {g.rotation=(g.rotation*Eigen::Quaterniond(Eigen::AngleAxisd(
        3.141592653589793,Eigen::Vector3d::UnitZ()))).normalized();}
    }
    scene_id_=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    trajectory_id_="gap_"+scene_id_;scene_valid_=true;requested_count_=req->count;
    res->accepted=true;res->message="Task selected; configuring selected gates, then planning in CMD";
    if(simulation_) {
      configuring_=true;configure_started_=std::chrono::steady_clock::now();
      auto request=std::make_shared<gap_msgs::srv::ConfigureGates::Request>();request->count=req->count;
      const auto selected_scene=scene_id_;
      configure_->async_send_request(request,[this,selected_scene](rclcpp::Client<gap_msgs::srv::ConfigureGates>::SharedFuture f) {
        if(selected_scene!=scene_id_ || !configuring_) {return;}
        configuring_=false;
        try {const auto r=f.get();if(!r->accepted) {throw std::runtime_error(r->message);}launchPlan();}
        catch(const std::exception &e) {scene_valid_=false;report(std::string("FAILED: scene selection: ")+e.what());}
      });
    } else {launchPlan();}
  }
  void visualize(const Result *result=nullptr)
  {
    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker clear;clear.action=clear.DELETEALL;out.markers.push_back(clear);
    int id=0;
    for(std::size_t index=0;index<selected_count_;++index) {
      const auto &g=tracked_[index].gate;
      for(int side=0;side<4;++side) {
        visualization_msgs::msg::Marker m;m.header.frame_id="world_nwu";m.header.stamp=now();
        m.ns="gates";m.id=id++;m.type=m.CUBE;m.action=m.ADD;
        Eigen::Vector3d offset=Eigen::Vector3d::Zero(),size;
        if(side<2) {offset.y()=(side==0?-1:1)*(g.width+g.frame_width)/2;
          size={g.thickness,g.frame_width,g.height+2*g.frame_width};}
        else {offset.z()=(side==2?-1:1)*(g.height+g.frame_width)/2;
          size={g.thickness,g.width,g.frame_width};}
        assign(m.pose.position,g.center+g.rotation*offset);assign(m.scale,size);
        m.pose.orientation.w=g.rotation.w();m.pose.orientation.x=g.rotation.x();
        m.pose.orientation.y=g.rotation.y();m.pose.orientation.z=g.rotation.z();
        m.color.r=0.2;m.color.g=0.7;m.color.b=1;m.color.a=0.8;out.markers.push_back(m);
      }
    }
    if(result) {
      visualization_msgs::msg::Marker line;line.header.frame_id="world_nwu";line.header.stamp=now();
      line.ns="planned_path";line.id=id++;line.type=line.LINE_STRIP;line.action=line.ADD;
      line.pose.orientation.w=1;line.scale.x=0.012;line.color.g=1;line.color.a=1;
      for(const auto &piece:result->pieces) {
        const int n=std::max(2,static_cast<int>(std::ceil(piece.duration/0.025)));
        for(int i=0;i<=n;++i) {
          const auto r=px4ctrl::externalReference(flatPoint(piece,piece.duration*i/n),options_.model);
          geometry_msgs::msg::Point point;assign(point,r.position);line.points.push_back(point);
          if(i%8==0) {
            visualization_msgs::msg::Marker body;body.header=line.header;body.ns="body_envelope";
            body.id=id++;body.type=body.SPHERE;body.action=body.ADD;body.pose.position=point;
            body.pose.orientation.w=r.attitude.w();body.pose.orientation.x=r.attitude.x();
            body.pose.orientation.y=r.attitude.y();body.pose.orientation.z=r.attitude.z();
            assign(body.scale,2*options_.body);body.color.r=1;body.color.g=0.7;body.color.a=0.18;
            if(!options_.vertices.empty()) {
              body.type=body.LINE_LIST;body.scale.x=0.005;body.scale.y=0;body.scale.z=0;body.color.a=0.5;
              for(const auto &edge:body_edges_) {for(auto index:{edge.first,edge.second}) {
                geometry_msgs::msg::Point vertex;assign(vertex,options_.vertices[index]);body.points.push_back(vertex);
              }}
            }
            out.markers.push_back(body);
          }
        }
      }
      out.markers.push_back(line);
    }
    markers_->publish(out);
  }
  void tick()
  {
    const double time=now().seconds();std::string why;
    if(last_visual_<0 || time-last_visual_>=0.5 || time<last_visual_) {
      visualize(displayed_plan_.get());last_visual_=time;
    }
    if(last_tick_>=0 && time<last_tick_) {
      scene_valid_=false;for(auto &t:tracked_) {t.stamp=-1;t.received=-1;}
      report("Clock reset; select a new plan");
    }
    last_tick_=time;
    if(configuring_ && std::chrono::duration<double>(std::chrono::steady_clock::now()-configure_started_).count()>5) {
      configuring_=false;scene_valid_=false;report("FAILED: simulator gate selection timed out");
    }
    if(scene_valid_ && !state_.command_mode) {scene_valid_=false;displayed_plan_.reset();}
    if(scene_valid_ && !snapshotMatches(why)) {scene_valid_=false;displayed_plan_.reset();report("Scene invalid: "+why);}
    gap_msgs::msg::SceneStatus scene;scene.header.stamp=now();scene.header.frame_id="world_nwu";
    scene.scene_id=scene_id_;scene.valid=scene_valid_;scene.reason=why;scenes_->publish(scene);
    if(worker_.valid() && worker_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      PlanTiming completed_timing;bool solved=false;
      try {
        const auto result=worker_.get();
        completed_timing=result.timing;solved=true;
        if(!scene_valid_ || !snapshotMatches(why)) {throw std::runtime_error("Scene changed during planning: "+why);}
        if(!state_.command_mode || !state_.hovering || time-state_received_>0.5) {throw std::runtime_error("Controller left CMD holding during planning");}
        gap_msgs::msg::PolynomialPlan m;m.header.stamp=now();m.header.frame_id="world_nwu";
        m.trajectory_id=trajectory_id_;m.scene_id=scene_id_;m.model=message(options_.model);
        for(const auto &p:result.pieces) {
          m.durations.push_back(p.duration);
          for(int row=0;row<3;++row) {for(int col=0;col<6;++col) {m.coefficients.push_back(p.coefficients(row,col));}}
        }
        displayed_plan_=std::make_shared<Result>(result);visualize(displayed_plan_.get());plans_->publish(m);
        std::string summary="PLANNED flight_duration="+std::to_string(result.duration)+" s, clearance="+
          std::to_string(result.min_clearance)+" m; path_length="+std::to_string(result.path_length)+" m endpoint_distance="+std::to_string(result.endpoint_distance)+" m backward_distance="+std::to_string(result.backward_distance)+" m; "+timingText(result.timing);
        for(const auto &crossing:result.crossings) {
          summary+="; "+crossing.id+" crossing_roll_deg="+std::to_string(crossing.roll_deg)+
            " crossing_tilt_deg="+std::to_string(crossing.tilt_deg);
        }
        report(summary+"; awaiting controller validation");
      } catch(const PlanError &e) {
        scene_valid_=false;report(std::string("FAILED: ")+e.what()+"; "+timingText(e.timing));
      } catch(const std::exception &e) {
        scene_valid_=false;report(std::string("FAILED: ")+e.what()+
          (solved?"; "+timingText(completed_timing):""));
      }
    }
  }
public:
  GapPlannerNode() : Node("gap_planner")
  {
    simulation_=declare_parameter("simulation",true);
    const double arm=declare_parameter("vehicle.arm",0.1216),prop=declare_parameter("vehicle.prop_radius",0.06475);
    const double height=declare_parameter("vehicle.height",0.10);
    // Enclose the entire radius-R, height-H cylinder, including top/bottom rims.
    const double horizontal=std::sqrt(2.0)*(arm+prop),vertical=height/std::sqrt(2.0);
    options_.body=vparam("vehicle.ellipsoid_half_axes",{horizontal,horizontal,vertical});
    const auto body_model=declare_parameter("vehicle.body_model",std::string("ellipsoid"));
    auto vertices=declare_parameter("vehicle.vertices",std::vector<double>{});
    if(body_model=="polytope") {
      if(vertices.empty()) {options_.vertices=boxVertices(arm+prop,height);}
      else {
        if(vertices.size()%3!=0) {throw std::invalid_argument("vehicle.vertices is a flat [x,y,z,...] array");}
        for(std::size_t i=0;i<vertices.size();i+=3) {options_.vertices.emplace_back(vertices[i],vertices[i+1],vertices[i+2]);}
      }
      validateVertices(options_.vertices);options_.body.setZero();
      for(const auto &v:options_.vertices) {options_.body=options_.body.cwiseMax(v.cwiseAbs());}
      body_edges_=hullEdges(options_.vertices);
    } else if(body_model!="ellipsoid") {throw std::invalid_argument("vehicle.body_model must be ellipsoid or polytope");}
    options_.model.mass=declare_parameter("vehicle.mass",0.811);
    options_.model.gravity=declare_parameter("vehicle.gravity",9.805);
    options_.model.drag=vparam("vehicle.drag_acceleration",{0.26/0.811,0.28/0.811,0.42/0.811});
    options_.model.lift=declare_parameter("vehicle.lift_acceleration",0.01/0.811);
    auto &limits=options_.execution_limits;limits.mass=options_.model.mass;limits.gravity=options_.model.gravity;
    limits.arm=arm;limits.arm_angle=declare_parameter("vehicle.arm_angle",M_PI/4);
    limits.inertia=vparam("vehicle.inertia",{0.00191,0.00237,0.00360});
    limits.torque_to_thrust=declare_parameter("vehicle.torque_to_thrust",0.01);
    limits.motor_min=declare_parameter("vehicle.motor_min",0.0);
    limits.motor_max=declare_parameter("vehicle.motor_max",25.0);
    limits.thrust_min=std::max(1e-6,4*limits.motor_min/limits.mass);limits.thrust_max=4*limits.motor_max/limits.mass;
    limits.rate_max=vparam("vehicle.execution_rate_max",{14,14,14});
    limits.angular_acceleration_max=declare_parameter("vehicle.angular_acceleration_max",100.0);
    limits.enforce_minimum_altitude=false;
    options_.model.heading=declare_parameter("planning.heading",0.0);
    options_.margin=declare_parameter("planning.margin",0.02);
    options_.optimization_buffer=declare_parameter("planning.optimization_buffer",0.002);
    options_.speed=declare_parameter("planning.speed_max",2.0);
    options_.rate=declare_parameter("planning.rate_max",6.0);
    options_.thrust_min=declare_parameter("planning.thrust_min",3.0);
    options_.thrust_max=declare_parameter("planning.thrust_max",18.0);
    options_.time_weight=declare_parameter("planning.time_weight",100.0);
    options_.solve_budget=declare_parameter("planning.solve_budget",5.0);
    for(const char *key:{"planning.tunnel_length","planning.path_length_weight","planning.backward_weight","planning.penalty"}) {
      if(get_node_parameters_interface()->get_parameter_overrides().count(key)) {
        RCLCPP_WARN(get_logger(),"%s is obsolete and ignored by the sparse gate planner; remove it from YAML",key);
      }
    }
    options_.tilt=declare_parameter("planning.tilt_max",1.52);
    goal_offset_x_=declare_parameter("mission.goal_offset_x",4.0);
    flight_height_=declare_parameter("mission.flight_height",1.0);
    if(!std::isfinite(goal_offset_x_) || goal_offset_x_<=0 || !std::isfinite(flight_height_) || flight_height_<=0) {
      throw std::invalid_argument("mission.goal_offset_x and flight_height must be finite and positive");
    }
    pose_timeout_=declare_parameter("mocap.timeout",0.3);
    move_tolerance_=declare_parameter("mocap.position_tolerance",0.01);
    angle_tolerance_=declare_parameter("mocap.angle_tolerance",0.02);
    mocap_frame_=declare_parameter("mocap.frame_id",std::string(""));
    world_translation_=vparam("mocap.world_translation",{0,0,0});world_rotation_=qparam("mocap.world_quaternion_wxyz");
    mocap_input_=std::make_unique<MocapInput>(mocap_frame_,world_translation_,world_rotation_,pose_timeout_);
    const auto ids=declare_parameter("gate_order",std::vector<std::string>{"gap_1","gap_2","gap_3"});
    if(ids.empty() || ids.size()>10) {throw std::invalid_argument("Configure 1..10 gates");}
    tracked_.resize(ids.size());
    for(std::size_t i=0;i<ids.size();++i) {
      auto &t=tracked_[i];t.gate.id=ids[i];const auto prefix="gates."+ids[i]+".";
      t.gate.width=declare_parameter(prefix+"width",0.9);t.gate.height=declare_parameter(prefix+"height",0.34);
      t.gate.thickness=declare_parameter(prefix+"thickness",0.04);t.gate.frame_width=declare_parameter(prefix+"frame_width",0.05);
      t.offset=vparam(prefix+"opening_offset",{0,0,0});t.offset_rotation=qparam(prefix+"opening_quaternion_wxyz");
      t.gate.center=vparam(prefix+"sim_position",{-0.9+static_cast<double>(i),0,1});
      const Eigen::Vector3d angles=vparam(prefix+"sim_rpy_deg",{0,0,0})*(3.141592653589793/180);
      t.gate.rotation=Eigen::AngleAxisd(angles.z(),Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(angles.y(),Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(angles.x(),Eigen::Vector3d::UnitX());
      // Simulation positions describe the rigid body, just like motion capture.
      t.gate.center+=t.gate.rotation*t.offset;t.gate.rotation=(t.gate.rotation*t.offset_rotation).normalized();
      const auto topic=declare_parameter(prefix+"topic",std::string("/")+ids[i]+"/pose");
      if(!simulation_) {
        RCLCPP_INFO(get_logger(),"Gate %s: %s [geometry_msgs/msg/PoseStamped], frame=%s",
          ids[i].c_str(),topic.c_str(),mocap_frame_.empty()?"<learn first valid message>":mocap_frame_.c_str());
        t.subscription=create_subscription<geometry_msgs::msg::PoseStamped>(topic,rclcpp::SensorDataQoS(),
          [this,i](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
            auto &t=tracked_[i];const double received=now().seconds();
            const bool was_locked=mocap_input_->frameLocked();MocapPose pose;
            if(!mocap_input_->accept(*m,received,t.stamp,t.offset,t.offset_rotation,pose,t.last_rejection)) {
              RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Rejecting %s: %s",
                t.gate.id.c_str(),t.last_rejection.c_str());return;
            }
            if(!was_locked) {
              RCLCPP_INFO(get_logger(),"Mocap parent frame locked to '%s' for all gates; YAML world transform remains unchanged",
                mocap_input_->frame().c_str());
            }
            t.stamp=pose.stamp;t.received=received;
            t.gate.center=pose.center;t.gate.rotation=pose.rotation;
          });
      }
    }
    configure_=create_client<gap_msgs::srv::ConfigureGates>("/gap/sim/configure");
    const auto latched=rclcpp::QoS(1).reliable().transient_local();
    plans_=create_publisher<gap_msgs::msg::PolynomialPlan>("/gap/polynomial",latched);
    scenes_=create_publisher<gap_msgs::msg::SceneStatus>("/gap/scene",10);
    markers_=create_publisher<visualization_msgs::msg::MarkerArray>("/gap/markers",latched);
    status_=create_publisher<std_msgs::msg::String>("/gap/planner_status",latched);
    state_sub_=create_subscription<gap_msgs::msg::ExecutionStatus>("/gap/execution",10,
      [this](gap_msgs::msg::ExecutionStatus::ConstSharedPtr m) {state_=*m;state_received_=now().seconds();});
    service_=create_service<gap_msgs::srv::PlanGaps>("/gap/plan",
      std::bind(&GapPlannerNode::request,this,std::placeholders::_1,std::placeholders::_2));
    timer_=create_wall_timer(std::chrono::milliseconds(100),std::bind(&GapPlannerNode::tick,this));
    visualize();report(simulation_?"SIMULATION starts empty; enter CMD then select count":"MOCAP mode; enter CMD, select the rigid bodies present for this task");
  }
};
int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<GapPlannerNode>());}
  catch(const std::exception &e) {std::cerr<<"gap_planner: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
