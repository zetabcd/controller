#include <px4ctrl/trajectory.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <stdexcept>

// Offline tool: no ROS initialization, publications or flight side effects.
int main(int argc, char ** argv)
{
  try {
    px4ctrl::AnalyticTrajectoryOptions o;
    px4ctrl::TrajectoryLimits l;
    std::string type = "figure_eight", output;
    double dt = .001, turns = o.turns;
    std::map<std::string, double *> fields{
      {"gravity", &o.gravity}, {"takeoff_height", &o.takeoff_height},
      {"takeoff_duration", &o.takeoff_duration},
      {"settle_duration", &o.settle_duration}, {"radius", &o.radius}, {"turns", &turns},
      {"speed", &o.speed}, {"ramp_duration", &o.ramp_duration}, {"centripetal_g", &o.centripetal_g},
      {"pitch", &o.pitch}, {"entry_duration", &o.entry_duration},
      {"exit_duration", &o.exit_duration},
      {"entry_distance", &o.entry_distance}, {"exit_distance", &o.exit_distance},
      {"connector_height", &o.connector_height},
      {"axis_transition_duration", &o.axis_transition_duration},
      {"length", &o.eight_length}, {"width", &o.eight_width}, {"dt", &dt},
      {"mass", &l.mass}, {"ix", &l.inertia.x()}, {"iy", &l.inertia.y()}, {"iz", &l.inertia.z()},
      {"arm", &l.arm}, {"arm_angle", &l.arm_angle}, {"torque_to_thrust", &l.torque_to_thrust},
      {"motor_min", &l.motor_min}, {"motor_max", &l.motor_max},
      {"thrust_min", &l.thrust_min}, {"thrust_max", &l.thrust_max},
      {"rate_x", &l.rate_max.x()}, {"rate_y", &l.rate_max.y()}, {"rate_z", &l.rate_max.z()},
      {"angular_acceleration", &l.angular_acceleration_max},
      {"minimum_relative_altitude", &l.minimum_relative_altitude}};
    for (int i = 1; i < argc; i += 2) {
      if (i + 1 >= argc) {throw std::invalid_argument("Arguments require --key value pairs");}
      const std::string key = argv[i], value = argv[i + 1];
      if (key == "--type") {type = value;} else if (key == "--output") {output = value;} else {
        if (key.rfind("--", 0) != 0 || !fields.count(key.substr(2))) {
          throw std::invalid_argument("Unknown argument " + key);
        }
        std::size_t used = 0;const double v = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(v)) {
          throw std::invalid_argument("Invalid value for " + key);
        }
        *fields.at(key.substr(2)) = v;
      }
    }
    if (turns < 1 || turns > 20 || turns != std::floor(turns)) {
      throw std::invalid_argument("turns must be integer 1..20");
    }
    o.turns = static_cast<int>(turns);l.gravity = o.gravity;
    if (type == "horizontal_circle") {
      o.path = px4ctrl::AnalyticPath::HorizontalCircle;
    } else if (type == "vertical_circle") {
      o.path = px4ctrl::AnalyticPath::VerticalCircle;
    } else if (type == "helix") {
      o.path = px4ctrl::AnalyticPath::Helix;
    } else if (type != "figure_eight") {
      throw std::invalid_argument("Unknown analytic trajectory type");
    }
    px4ctrl::AnalyticTrajectory trajectory(o);
    const auto report = px4ctrl::auditTrajectory(trajectory, l, dt);
    if (!output.empty()) {
      std::ofstream file(output);file << std::setprecision(15);
      if (!file) {throw std::runtime_error("Cannot open output CSV");}
      file <<
        "t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz,qw,qx,qy,qz,thrust_acc,wx,wy,wz,alphax,alphay,alphaz\n";
      const int n = static_cast<int>(std::ceil(trajectory.duration() / dt));
      for (int i = 0; i <= n; ++i) {
        const double time = trajectory.duration() * i / n;const auto r = trajectory.evaluate(time);
        file << time;
        for (const auto & v:{r.position, r.velocity, r.acceleration, r.jerk, r.snap}) {
          file << ',' << v.x() << ',' << v.y() << ',' << v.z();
        }
        file << ',' << r.attitude.w() << ',' << r.attitude.x() << ',' << r.attitude.y() << ',' <<
          r.attitude.z() << ',' << r.thrust_acceleration;
        for (const auto & v:{r.body_rate, r.body_acceleration}) {
          file << ',' << v.x() << ',' << v.y() << ',' << v.z();
        }
        file << '\n';
      }
      if (!file) {throw std::runtime_error("CSV write failed");}
    }
    std::cout << std::setprecision(12) << "{\"valid\":" << (report.valid ? "true" : "false")
              << ",\"duration\":" << trajectory.duration() << ",\"main_start\":" <<
      trajectory.mainStart()
              << ",\"main_end\":" << trajectory.mainEnd() << ",\"max_speed\":" << report.max_speed
              << ",\"min_thrust_acc\":" << report.min_thrust << ",\"max_thrust_acc\":" <<
      report.max_thrust
              << ",\"max_rate\":" << report.max_rate << ",\"max_alpha\":" <<
      report.max_angular_acceleration
              << ",\"min_motor\":" << report.min_motor << ",\"max_motor\":" << report.max_motor
              << ",\"min_altitude\":" << report.min_altitude << ",\"first_failure_time\":" <<
      report.first_failure_time << "}\n";
    if (!report.valid) {std::cerr << report.reason << '\n';return 2;}
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';return 1;
  }
}
