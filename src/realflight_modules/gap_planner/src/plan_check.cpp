#include <gap_planner/core.h>
#include <chrono>
#include <iostream>
#include <yaml-cpp/yaml.h>
int main(int argc,char **argv)
{
  try {
    const bool yaml=argc>1 && std::string(argv[1])=="--yaml";
    const int count=yaml?(argc>3?std::stoi(argv[3]):1):(argc>1?std::stoi(argv[1]):1);
    const double angle=!yaml && argc>2?std::stod(argv[2]):30;
    gap_planner::Options o;o.time_weight=100;o.model.drag={0.26/0.811,0.28/0.811,0.42/0.811};o.model.lift=0.01/0.811;
    // count angle [nominal|noisy] [ellipsoid|polytope] [width] [height] [speed] [reserved]
    if(!yaml && argc>4 && std::string(argv[4])=="polytope") {
      o.vertices=gap_planner::boxVertices(0.1216+0.06475,0.10);o.body={0.18635,0.18635,0.05};
    }
    if(!yaml && argc>7) {o.speed=std::stod(argv[7]);}
    std::vector<gap_planner::Gate> gates;
    for(int i=0;i<3;++i) {
      gap_planner::Gate g;g.id="gap_"+std::to_string(i+1);g.center={-0.9+i,0,1};g.width=0.9;g.height=0.34;
      if(!yaml && argc>5) {g.width=std::stod(argv[5]);}if(!yaml && argc>6) {g.height=std::stod(argv[6]);}
      g.rotation=Eigen::AngleAxisd((i==1?-angle:angle)*3.141592653589793/180,Eigen::Vector3d::UnitX());gates.push_back(g);
    }
    Eigen::Vector3d yaml_start(-1.65,0,1.05),yaml_goal(2.35,0,1.05);
    bool returning=false;
    if(yaml) {
      const auto root=YAML::LoadFile(argv[2])["gap_planner"]["ros__parameters"];
      const auto p=root["planning"],v=root["vehicle"],mission=root["mission"];
      auto read=[](const YAML::Node &n,const char *key,double fallback) {return n[key]?n[key].as<double>():fallback;};
      auto vec=[](const YAML::Node &n) {return Eigen::Vector3d(n[0].as<double>(),n[1].as<double>(),n[2].as<double>());};
      auto quat=[](const YAML::Node &n) {return n?Eigen::Quaterniond(n[0].as<double>(),n[1].as<double>(),n[2].as<double>(),n[3].as<double>()).normalized():Eigen::Quaterniond::Identity();};
      o.margin=read(p,"margin",o.margin);o.optimization_buffer=read(p,"optimization_buffer",o.optimization_buffer);
      o.speed=read(p,"speed_max",o.speed);o.rate=read(p,"rate_max",o.rate);
      o.thrust_min=read(p,"thrust_min",o.thrust_min);o.thrust_max=read(p,"thrust_max",o.thrust_max);
      o.solve_budget=read(p,"solve_budget",o.solve_budget);o.model.heading=read(p,"heading",0);
      o.tilt=read(p,"tilt_max",o.tilt);o.time_weight=read(p,"time_weight",o.time_weight);
      double radius=0.1216+0.06475;
      if(argc>5) {
        const auto c=YAML::LoadFile(argv[5])["px4ctrl_node"]["ros__parameters"];
        o.model.mass=c["uav"]["mass"].as<double>();o.model.gravity=c["gra"].as<double>();
        radius=c["uav"]["l"].as<double>()+c["uav"]["rp"].as<double>();
        const bool drag=!c["mpc"]["drag_compensation"] || c["mpc"]["drag_compensation"].as<bool>();
        int k=0;for(const char *key:{"kdx","kdy","kdz"}) {o.model.drag[k++]=drag?c["aero"][key].as<double>()/o.model.mass:0;}
        auto &limits=o.execution_limits;const auto u=c["uav"],m=c["motor"],l=c["trajectory"]["limits"];
        limits.mass=o.model.mass;limits.gravity=o.model.gravity;limits.arm=u["l"].as<double>();
        limits.inertia={u["Jvx"].as<double>(),u["Jvy"].as<double>(),u["Jvz"].as<double>()};
        limits.arm_angle=u["beta_deg"].as<double>()*M_PI/180;
        const double rp=u["rp"].as<double>(),ct=m["Ct_c"].as<double>()*4*c["aero"]["rho"].as<double>()*std::pow(rp,4)/(M_PI*M_PI);
        const double speed_min=m["rc2speed_c"].as<double>(),speed_max=m["rc2speed_a"].as<double>()+m["rc2speed_b"].as<double>()+speed_min;
        limits.torque_to_thrust=2*rp*m["Cq_c"].as<double>()/m["Ct_c"].as<double>();
        limits.motor_min=ct*speed_min*speed_min;limits.motor_max=ct*speed_max*speed_max*read(l,"motor_fraction",0.7);
        limits.thrust_min=std::max(1e-6,4*limits.motor_min/limits.mass);limits.thrust_max=4*limits.motor_max/limits.mass;
        limits.angular_acceleration_max=read(l,"angular_acceleration",100);
        if(c["mpc"]["body_rate_max"]) {limits.rate_max=vec(c["mpc"]["body_rate_max"]);}
        limits.enforce_minimum_altitude=false;
        o.model.lift=drag?c["aero"]["kh"].as<double>()/o.model.mass:0;
      }
      const double height=read(v,"height",0.1);
      o.body={std::sqrt(2.0)*radius,std::sqrt(2.0)*radius,height/std::sqrt(2.0)};
      if(v["ellipsoid_half_axes"]) {o.body=vec(v["ellipsoid_half_axes"]);}
      if(v["body_model"] && v["body_model"].as<std::string>()=="polytope") {
        o.vertices.clear();
        if(v["vertices"]) {
          const auto values=v["vertices"].as<std::vector<double>>();
          if(values.size()%3) {throw std::invalid_argument("vertices must be triples");}
          for(size_t i=0;i<values.size();i+=3) {o.vertices.emplace_back(values[i],values[i+1],values[i+2]);}
        } else {o.vertices=gap_planner::boxVertices(radius,height);}
        gap_planner::validateVertices(o.vertices);o.body.setZero();
        for(const auto &point:o.vertices) {o.body=o.body.cwiseMax(point.cwiseAbs());}
      }
      gates.clear();
      for(const auto &name:root["gate_order"]) {
        gap_planner::Gate g;g.id=name.as<std::string>();const auto n=root["gates"][g.id];
        g.width=n["width"].as<double>();g.height=n["height"].as<double>();g.thickness=n["thickness"].as<double>();
        g.frame_width=read(n,"frame_width",0.05);g.center=vec(n["sim_position"]);
        const Eigen::Vector3d a=vec(n["sim_rpy_deg"])*3.141592653589793/180;
        g.rotation=Eigen::AngleAxisd(a.z(),Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(a.y(),Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(a.x(),Eigen::Vector3d::UnitX());
        if(n["opening_offset"]) {g.center+=g.rotation*vec(n["opening_offset"]);}
        g.rotation=g.rotation*quat(n["opening_quaternion_wxyz"]);gates.push_back(g);
      }
      yaml_start.z()=0.05+read(mission,"flight_height",1);yaml_goal=yaml_start+Eigen::Vector3d(read(mission,"goal_offset_x",4),0,0);
      returning=argc>4 && std::string(argv[4])=="return";
    }
    if(count<1 || count>static_cast<int>(gates.size())) {throw std::invalid_argument("count must be 1..3");}
    Eigen::Vector3d start=argc>3 && std::string(argv[3])=="noisy"?Eigen::Vector3d(-1.642,0.009,1.047):Eigen::Vector3d(-1.65,0,1.05);
    Eigen::Vector3d goal(2.35,0,1.05);gates.resize(count);
    if(yaml) {start=yaml_start;goal=yaml_goal;}
    if(returning || (!yaml && argc>9 && std::string(argv[9])=="return")) {
      std::swap(start,goal);std::reverse(gates.begin(),gates.end());
      for(auto &g:gates) {g.rotation=g.rotation*Eigen::Quaterniond(Eigen::AngleAxisd(3.141592653589793,Eigen::Vector3d::UnitZ()));}
    }
    if(yaml && argc>8) {start+=Eigen::Vector3d(std::stod(argv[6]),std::stod(argv[7]),std::stod(argv[8]));}
    const auto r=gap_planner::plan(gates,count,start,goal,o);
    std::cout<<"PASS count="<<count<<" pieces="<<r.pieces.size()<<" duration="<<r.duration
      <<" clearance="<<r.min_clearance<<" speed="<<r.max_speed<<" rate="<<r.max_rate
      <<" path_length="<<r.path_length<<" backward_distance="<<r.backward_distance<<" endpoint_distance="<<r.endpoint_distance<<" "<<gap_planner::timingText(r.timing)<<"\n";
    for(const auto &g:r.crossings) {std::cout<<g.id<<" crossing_roll_deg="<<g.roll_deg<<" crossing_tilt_deg="<<g.tilt_deg<<"\n";}
  } catch(const gap_planner::PlanError &e) {
    std::cerr<<"REJECTED: "<<e.what()<<"; "<<gap_planner::timingText(e.timing)<<"\n";return 1;
  } catch(const std::exception &e) {std::cerr<<"REJECTED: "<<e.what()<<"\n";return 1;}
}
