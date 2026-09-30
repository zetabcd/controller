#include <px4ctrl/omtraj.h>
#include <uav_utils/project_paths.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
geometry_msgs::msg::Point point(const Eigen::Vector3d &value)
{
  geometry_msgs::msg::Point result;
  result.x = value.x();
  result.y = value.y();
  result.z = value.z();
  return result;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F)
{
  std_msgs::msg::ColorRGBA result;
  result.r = r;
  result.g = g;
  result.b = b;
  result.a = a;
  return result;
}
}  // namespace

class OmTrajectoryVisualizer : public rclcpp::Node
{
public:
  bool succeeded() const {return result_.success;}
  OmTrajectoryVisualizer()
  : Node("omtraj_visualizer")
  {
    declare_parameter<bool>("exit_after_solve", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    marker_topic_ = declare_parameter<std::string>(
      "visualization.marker_topic", "omtraj/markers");
    vehicle_publish_rate_hz_ = std::max(
      1.0, declare_parameter<double>("visualization.vehicle_publish_rate_hz", 30.0));
    vehicle_playback_speed_ = std::max(
      0.01, declare_parameter<double>("visualization.vehicle_playback_speed", 1.0));
    final_display_samples_ = std::max(
      2, static_cast<int>(
        declare_parameter<int>("visualization.final_display_samples", 240)));
    thrust_arrow_stride_ = std::max(
      1, static_cast<int>(
        declare_parameter<int>("visualization.thrust_arrow_stride", 1)));
    thrust_arrow_scale_ = std::max(
      0.0, declare_parameter<double>("visualization.thrust_arrow_scale", 0.03));
    trajectory_width_ = std::max(
      0.005, declare_parameter<double>("visualization.trajectory_width", 0.055));
    save_trajectory_ = declare_parameter<bool>("output.save_trajectory", true);
    trajectory_output_file_ = declare_parameter<std::string>(
      "output.trajectory_file", "datalog/omtraj/omtraj_manifold_v2.csv");
    if (save_trajectory_) {
      trajectory_output_file_ = uav_utils::projectPath(
        trajectory_output_file_.empty() ? "datalog/omtraj/omtraj_manifold_v2.csv" : trajectory_output_file_,
        ament_index_cpp::get_package_share_directory("px4ctrl")).string();
    }

    const int waypoint_count = std::max(
      0, static_cast<int>(declare_parameter<int>("waypoint_count", 3)));
    const double waypoint_tolerance = std::max(
      0.0, declare_parameter<double>("waypoint_tolerance", 0.12));
    const double waypoint_margin = declare_parameter<double>("tracking.waypoint_margin", 0.0);
    if (!std::isfinite(waypoint_margin) || waypoint_margin < 0 || waypoint_margin >= waypoint_tolerance) {
      throw std::invalid_argument("Waypoint margin must be nonnegative and smaller than task tolerance");
    }
    waypoints_.reserve(static_cast<std::size_t>(waypoint_count));
    for (int i = 0; i < waypoint_count; ++i) {
      OmTrajectoryWaypoint waypoint;
      // ROS 2 parameters do not support arrays-of-arrays.  A named vector per
      // point preserves the desired [x,y,z] grouping without parsing strings.
      waypoint.position = vector3Parameter(
        "waypoints.point_" + std::to_string(i), {});
      waypoint.tolerance = waypoint_tolerance - waypoint_margin;
      waypoint.maximum_tilt = declare_parameter<double>("waypoints.tilt_" + std::to_string(i), M_PI);
      waypoints_.push_back(waypoint);
      RCLCPP_INFO(
        get_logger(), "Loaded waypoint[%d] = [%.3f, %.3f, %.3f], tolerance=%.3f m",
        i, waypoint.position.x(), waypoint.position.y(), waypoint.position.z(),
        waypoint.tolerance);
    }

    OmTrajectoryBoundary initial;
    OmTrajectoryBoundary terminal;
    initial.position = vector3Parameter("initial.position", {0.0, 0.0, 0.0});
    initial.velocity = vector3Parameter("initial.velocity", {0.0, 0.0, 0.0});
    terminal.position = vector3Parameter("terminal.position", {4.0, 0.0, 1.0});
    terminal.velocity = vector3Parameter("terminal.velocity", {0.0, 0.0, 0.0});
    initial.attitude = Eigen::AngleAxisd(
      declare_parameter<double>("initial.yaw", 0.0), Eigen::Vector3d::UnitZ());
    terminal.attitude = Eigen::AngleAxisd(
      declare_parameter<double>("terminal.yaw", 0.0), Eigen::Vector3d::UnitZ());

    OmTrajectoryOptions options;
    options.intervals = declare_parameter<int>("optimizer.intervals", options.intervals);
    options.max_scp_iterations = declare_parameter<int>("optimizer.max_scp_iterations", options.max_scp_iterations);
    options.integration_substeps = declare_parameter<int>("optimizer.integration_substeps", options.integration_substeps);
    options.scp_backtracking_steps = declare_parameter<int>("optimizer.scp_backtracking_steps", options.scp_backtracking_steps);
    options.optimize_total_time = declare_parameter<bool>("optimizer.optimize_total_time", options.optimize_total_time);
    options.enforce_hover_boundary_input = declare_parameter<bool>("optimizer.enforce_hover_boundary_input", options.enforce_hover_boundary_input);
    options.store_history = declare_parameter<bool>("optimizer.store_history", options.store_history);
    options.minimum_time = declare_parameter<double>("optimizer.minimum_time", options.minimum_time);
    options.maximum_time = declare_parameter<double>("optimizer.maximum_time", options.maximum_time);
    options.initial_speed = declare_parameter<double>("optimizer.initial_speed", options.initial_speed);
    options.thrust_acceleration_min = declare_parameter<double>("optimizer.thrust_acceleration_min", options.thrust_acceleration_min);
    options.thrust_acceleration_max = declare_parameter<double>("optimizer.thrust_acceleration_max", options.thrust_acceleration_max);
    options.position_trust_region = declare_parameter<double>("optimizer.position_trust_region", options.position_trust_region);
    options.velocity_trust_region = declare_parameter<double>("optimizer.velocity_trust_region", options.velocity_trust_region);
    options.attitude_trust_region = declare_parameter<double>("optimizer.attitude_trust_region", options.attitude_trust_region);
    options.thrust_trust_region = declare_parameter<double>("optimizer.thrust_trust_region", options.thrust_trust_region);
    options.time_trust_region = declare_parameter<double>("optimizer.time_trust_region", options.time_trust_region);
    options.progress_trust_region = declare_parameter<double>("optimizer.progress_trust_region", options.progress_trust_region);
    options.state_regularization = declare_parameter<double>("optimizer.state_regularization", options.state_regularization);
    options.progress_regularization = declare_parameter<double>("optimizer.progress_regularization", options.progress_regularization);
    options.maximum_virtual_control_weight = declare_parameter<double>("optimizer.maximum_virtual_control_weight", options.maximum_virtual_control_weight);
    options.virtual_control_weight = declare_parameter<double>("optimizer.virtual_control_weight", options.virtual_control_weight);
    options.cstc_merit_weight = declare_parameter<double>("optimizer.cstc_merit_weight", options.cstc_merit_weight);
    options.convergence_tolerance = declare_parameter<double>("optimizer.convergence_tolerance", options.convergence_tolerance);
    options.position_tolerance = declare_parameter<double>("optimizer.position_tolerance", options.position_tolerance);
    options.velocity_tolerance = declare_parameter<double>("optimizer.velocity_tolerance", options.velocity_tolerance);
    options.attitude_tolerance = declare_parameter<double>("optimizer.attitude_tolerance", options.attitude_tolerance);
    options.constraint_tolerance = declare_parameter<double>("optimizer.constraint_tolerance", options.constraint_tolerance);
    options.cstc_tolerance = declare_parameter<double>("optimizer.cstc_tolerance", options.cstc_tolerance);
    options.cstc_relaxation_initial = declare_parameter<double>("optimizer.cstc_relaxation_initial", options.cstc_relaxation_initial);
    options.cstc_relaxation_decay = declare_parameter<double>("optimizer.cstc_relaxation_decay", options.cstc_relaxation_decay);
    options.body_rate_max = vector3Parameter("optimizer.body_rate_max", {options.body_rate_max.x(), options.body_rate_max.y(), options.body_rate_max.z()});
    options.body_rate_trust_region = vector3Parameter("optimizer.body_rate_trust_region", {options.body_rate_trust_region.x(), options.body_rate_trust_region.y(), options.body_rate_trust_region.z()});
    options.model.linear_drag = vector3Parameter("model.linear_drag", {options.model.linear_drag.x(), options.model.linear_drag.y(), options.model.linear_drag.z()});
    options.tracking.rate_max = vector3Parameter("tracking.rate_max", {options.tracking.rate_max.x(), options.tracking.rate_max.y(), options.tracking.rate_max.z()});
    options.tracking.angular_acceleration_max = vector3Parameter("tracking.angular_acceleration_max", {options.tracking.angular_acceleration_max.x(), options.tracking.angular_acceleration_max.y(), options.tracking.angular_acceleration_max.z()});
    options.tracking.inertia = vector3Parameter("tracking.inertia", {options.tracking.inertia.x(), options.tracking.inertia.y(), options.tracking.inertia.z()});
    options.model.gravity = declare_parameter<double>("model.gravity", options.model.gravity);
    options.model.horizontal_lift = declare_parameter<double>("model.horizontal_lift", options.model.horizontal_lift);
    options.tracking.enabled = declare_parameter<bool>("tracking.enabled", options.tracking.enabled);
    options.tracking.motor_constraints = declare_parameter<bool>("tracking.motor_constraints", options.tracking.motor_constraints);
    options.tracking.thrust_min = declare_parameter<double>("tracking.thrust_min", options.tracking.thrust_min);
    options.tracking.thrust_max = declare_parameter<double>("tracking.thrust_max", options.tracking.thrust_max);
    options.tracking.thrust_time_constant = declare_parameter<double>("tracking.thrust_time_constant", options.tracking.thrust_time_constant);
    options.tracking.thrust_slew_max = declare_parameter<double>("tracking.thrust_slew_max", options.tracking.thrust_slew_max);
    options.tracking.speed_max = declare_parameter<double>("tracking.speed_max", options.tracking.speed_max);
    options.tracking.maximum_tilt = declare_parameter<double>("tracking.maximum_tilt", options.tracking.maximum_tilt);
    options.tracking.minimum_relative_altitude = declare_parameter<double>("tracking.minimum_relative_altitude", options.tracking.minimum_relative_altitude);
    options.tracking.mass = declare_parameter<double>("tracking.mass", options.tracking.mass);
    options.tracking.arm = declare_parameter<double>("tracking.arm", options.tracking.arm);
    options.tracking.arm_angle = declare_parameter<double>("tracking.arm_angle", options.tracking.arm_angle);
    options.tracking.torque_to_thrust = declare_parameter<double>("tracking.torque_to_thrust", options.tracking.torque_to_thrust);
    options.tracking.motor_min = declare_parameter<double>("tracking.motor_min", options.tracking.motor_min);
    options.tracking.motor_max = declare_parameter<double>("tracking.motor_max", options.tracking.motor_max);

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).transient_local().reliable());
    RCLCPP_INFO(
      get_logger(), "Atomic RViz MarkerArray topic: %s (reliable, transient_local)",
      marker_publisher_->get_topic_name());
    RCLCPP_INFO(
      get_logger(), "Solving trajectory with %d ordered waypoint(s), N=%d ...",
      waypoint_count, options.intervals);
    options.progress_callback = [this, max_iterations = options.max_scp_iterations](
        const OmTrajectoryIteration &progress) {
        ++live_progress_count_;
        if (progress.states.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "[SOLVE] global=%zu SCP=%d/%d %s | "
            "step=%.3fs QP=%.3fs elapsed=%.3fs T=%.3fs",
            live_progress_count_,
            progress.scp_iteration, max_iterations,
            progress.solver_status.c_str(), progress.step_solve_time,
            progress.qp_solve_time, progress.elapsed_solve_time, progress.total_time);
          publishLiveOptimizationFrame(progress);
          return;
        }
        RCLCPP_INFO(
          get_logger(),
          "[SOLVE] global=%zu SCP=%d/%d | "
          "step=%.3fs QP=%.3fs elapsed=%.3fs T=%.3fs | "
          "update=%.2e dyn=%.2e virtual=%.2e wp=%.2e order=%.2e | %s",
          live_progress_count_,
          progress.scp_iteration, max_iterations,
          progress.step_solve_time, progress.qp_solve_time, progress.elapsed_solve_time,
          progress.total_time, progress.maximum_update,
          progress.maximum_dynamics_defect, progress.maximum_virtual_control,
          progress.maximum_waypoint_residual, progress.maximum_order_residual,
          progress.solver_status.c_str());
        publishLiveOptimizationFrame(progress);
      };
    OmTrajectoryOptimizer optimizer(options);
    result_ = optimizer.optimize(initial, terminal, waypoints_);
    if (result_.success && result_.total_time > 0.0) {
      final_display_states_.reserve(static_cast<std::size_t>(final_display_samples_ + 1));
      for (int i = 0; i <= final_display_samples_; ++i) {
        final_display_states_.push_back(OmTrajectoryOptimizer::sample(
          result_, result_.total_time * static_cast<double>(i) /
          static_cast<double>(final_display_samples_)));
      }
    }
    RCLCPP_INFO(
      get_logger(), "Optimization %s (converged=%s): %s, T=%.3f s, "
      "solve wall time=%.3f s, final-attempt SCP steps=%d, stored frames=%zu",
      result_.success ? "succeeded" : "failed",
      result_.converged ? "true" : "false", result_.status.c_str(),
      result_.total_time, result_.total_solve_time, result_.scp_iterations,
      result_.history.size());
    if (save_trajectory_ && result_.success) {
      std::string save_error;
      if (!saveOmTrajectoryCsv(result_, trajectory_output_file_, &save_error)) {
        RCLCPP_ERROR(
          get_logger(), "Failed to save optimized trajectory to '%s': %s",
          trajectory_output_file_.c_str(), save_error.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "Saved optimized control trajectory to: %s",
          trajectory_output_file_.c_str());
      }
    }
    for (std::size_t i = 0; i < result_.waypoint_times.size(); ++i) {
      RCLCPP_INFO(get_logger(), "waypoint[%zu] selected time: %.3f s", i,
        result_.waypoint_times[i]);
    }
    if (result_.success && result_.states.size() > 1U) {
      const double grid_dt = result_.total_time /
        static_cast<double>(result_.states.size() - 1U);
      RCLCPP_INFO(
        get_logger(),
        "Trajectory grid: %zu states, dt=%.4f s (%.1f Hz); MPC resamples it at control dt",
        result_.states.size(), grid_dt, 1.0 / grid_dt);
      double maximum_speed = 0.0;
      double maximum_tilt = 0.0;
      double minimum_thrust = std::numeric_limits<double>::infinity();
      double maximum_thrust = 0.0;
      Eigen::Vector3d maximum_body_rate = Eigen::Vector3d::Zero();
      double maximum_thrust_rate = 0.0;
      double maximum_body_rate_acceleration = 0.0;
      for (std::size_t k = 0; k < result_.states.size(); ++k) {
        const auto &state = result_.states[k];
        maximum_speed = std::max(maximum_speed, state.velocity.norm());
        const Eigen::Vector3d body_z = state.attitude * Eigen::Vector3d::UnitZ();
        maximum_tilt = std::max(
          maximum_tilt, std::acos(std::clamp(body_z.z(), -1.0, 1.0)));
        minimum_thrust = std::min(minimum_thrust, state.thrust_acceleration);
        maximum_thrust = std::max(maximum_thrust, state.thrust_acceleration);
        maximum_body_rate = maximum_body_rate.cwiseMax(state.body_rate.cwiseAbs());
        if (k > 0U) {
          const double sample_dt = state.time - result_.states[k - 1U].time;
          if (sample_dt > 1.0e-9) {
            maximum_thrust_rate = std::max(
              maximum_thrust_rate,
              std::abs(state.thrust_acceleration -
              result_.states[k - 1U].thrust_acceleration) / sample_dt);
            maximum_body_rate_acceleration = std::max(
              maximum_body_rate_acceleration,
              (state.body_rate - result_.states[k - 1U].body_rate)
              .lpNorm<Eigen::Infinity>() / sample_dt);
          }
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "Trajectory limits: vmax=%.3f m/s, tilt_max=%.1f deg, aT=[%.3f, %.3f] m/s^2, "
        "|omega|max=[%.3f, %.3f, %.3f] rad/s, "
        "|dot(aT)|max=%.3f m/s^3, |dot(omega)|inf,max=%.3f rad/s^2",
        maximum_speed, 180.0 * maximum_tilt / M_PI, minimum_thrust, maximum_thrust,
        maximum_body_rate.x(), maximum_body_rate.y(), maximum_body_rate.z(),
        maximum_thrust_rate, maximum_body_rate_acceleration);
      const double thrust_utilization = maximum_thrust /
        std::max(options.tracking.enabled ? std::min(options.thrust_acceleration_max,
        options.tracking.thrust_max) : options.thrust_acceleration_max, 1.0e-9);
      const double body_rate_utilization =
        (maximum_body_rate.array() /
        (options.tracking.enabled ? options.body_rate_max.cwiseMin(options.tracking.rate_max) :
        options.body_rate_max).cwiseMax(Eigen::Vector3d::Constant(1.0e-9)).array()).maxCoeff();
      RCLCPP_INFO(
        get_logger(), "Control-bound utilization: thrust=%.1f%%, body-rate=%.1f%%",
        100.0 * thrust_utilization, 100.0 * body_rate_utilization);
      RCLCPP_INFO(
        get_logger(), "Final residuals: update=%.3e, nonlinear=%.3e, virtual=%.3e, waypoint=%.3e, "
        "order=%.3e",
        result_.maximum_update, result_.maximum_dynamics_defect, result_.maximum_virtual_control,
        result_.maximum_waypoint_residual, result_.maximum_order_residual);
    }

    RCLCPP_INFO(
      get_logger(), "Optimization finished after %zu live step(s); "
      "starting red final trajectory and looping simulated vehicle",
      live_progress_count_);
    publishVehicleFrame(0.0);
    last_timer_time_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / vehicle_publish_rate_hz_),
      std::bind(&OmTrajectoryVisualizer::publishFrame, this));
  }

private:
  Eigen::Vector3d vector3Parameter(
    const std::string &name, const std::vector<double> &default_value)
  {
    const auto values = declare_parameter<std::vector<double>>(name, default_value);
    if (values.size() != 3U) {
      throw std::invalid_argument(name + " must contain exactly three numbers");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  visualization_msgs::msg::Marker baseMarker(
    int id, const std::string &name_space, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = name_space;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void appendWaypoints(visualization_msgs::msg::MarkerArray *array)
  {
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      auto waypoint = baseMarker(
        static_cast<int>(i), "ordered_waypoints", visualization_msgs::msg::Marker::SPHERE);
      waypoint.pose.position = point(waypoints_[i].position);
      const double diameter = std::max(0.06, 2.0 * waypoints_[i].tolerance);
      waypoint.scale.x = diameter;
      waypoint.scale.y = diameter;
      waypoint.scale.z = diameter;
      waypoint.color = color(0.75F, 0.1F, 0.7F, 0.55F);
      array->markers.push_back(std::move(waypoint));

      auto label = baseMarker(
        static_cast<int>(i), "waypoint_labels",
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      label.pose.position = point(waypoints_[i].position + Eigen::Vector3d(0.0, 0.0, 0.22));
      label.scale.z = 0.16;
      label.color = color(0.08F, 0.08F, 0.08F);
      label.text = "P" + std::to_string(i + 1);
      array->markers.push_back(std::move(label));
    }
  }

  void appendTrajectoryAndThrust(
    const std::vector<OmTrajectoryState,
      Eigen::aligned_allocator<OmTrajectoryState>> &states,
    const std_msgs::msg::ColorRGBA &trajectory_color,
    visualization_msgs::msg::MarkerArray *array)
  {
    auto trajectory = baseMarker(
      0, "trajectory", visualization_msgs::msg::Marker::LINE_STRIP);
    trajectory.scale.x = trajectory_width_;
    trajectory.color = trajectory_color;
    trajectory.points.reserve(states.size());
    for (const auto &state : states) {
      trajectory.points.push_back(point(state.position));
    }
    array->markers.push_back(std::move(trajectory));

    // All arrows are placed in the same MarkerArray as the curve, so RViz
    // receives one atomic frame. Direction is R*e3 and length is scale*a_T.
    int arrow_id = 0;
    for (std::size_t k = 0; k < states.size();
      k += static_cast<std::size_t>(thrust_arrow_stride_))
    {
      const auto &state = states[k];
      auto arrow = baseMarker(
        arrow_id++, "body_z_thrust", visualization_msgs::msg::Marker::ARROW);
      const Eigen::Vector3d body_z = state.attitude * Eigen::Vector3d::UnitZ();
      arrow.points.push_back(point(state.position));
      arrow.points.push_back(point(
        state.position + thrust_arrow_scale_ * state.thrust_acceleration * body_z));
      arrow.scale.x = 0.014;
      arrow.scale.y = 0.035;
      arrow.scale.z = 0.05;
      arrow.color = color(0.015F, 0.015F, 0.015F, 0.92F);
      array->markers.push_back(std::move(arrow));
    }
  }

  void appendOptimizationStatus(
    const OmTrajectoryIteration &iteration,
    visualization_msgs::msg::MarkerArray *array)
  {
    auto status = baseMarker(
      0, "status", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    status.pose.position = point(iteration.states.front().position +
      Eigen::Vector3d(0.0, 0.0, 1.0));
    status.scale.z = 0.18;
    status.color = color(0.2F, 0.08F, 0.02F);
    std::ostringstream text;
    text << "OPTIMIZATION LIVE (orange)  global step " << live_progress_count_
         << "  SCP " << iteration.scp_iteration
         << "  T=" << std::fixed << std::setprecision(2) << iteration.total_time << "s"
         << "\nstep=" << std::fixed << std::setprecision(3) << iteration.step_solve_time
         << "s  QP=" << iteration.qp_solve_time << "s  elapsed="
         << iteration.elapsed_solve_time << "s"
         << "\ndyn=" << std::scientific << std::setprecision(1)
         << iteration.maximum_dynamics_defect
         << "  update=" << iteration.maximum_update
         << "  virtual=" << iteration.maximum_virtual_control
         << "  wp=" << iteration.maximum_waypoint_residual
         << "  order=" << iteration.maximum_order_residual;
    status.text = text.str();
    array->markers.push_back(std::move(status));
  }

  void appendVehicle(
    const OmTrajectoryState &state,
    visualization_msgs::msg::MarkerArray *array)
  {
    // A non-symmetric cuboid makes roll, pitch and yaw visually distinguishable.
    auto vehicle = baseMarker(
      0, "simulated_vehicle", visualization_msgs::msg::Marker::CUBE);
    vehicle.pose.position = point(state.position);
    vehicle.pose.orientation.x = state.attitude.x();
    vehicle.pose.orientation.y = state.attitude.y();
    vehicle.pose.orientation.z = state.attitude.z();
    vehicle.pose.orientation.w = state.attitude.w();
    vehicle.scale.x = 0.36;
    vehicle.scale.y = 0.24;
    vehicle.scale.z = 0.12;
    vehicle.color = color(0.08F, 0.35F, 0.95F, 0.9F);
    array->markers.push_back(std::move(vehicle));

    auto status = baseMarker(
      0, "status", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    status.pose.position = point(state.position + Eigen::Vector3d(0.0, 0.0, 0.32));
    status.scale.z = 0.16;
    status.color = color(0.04F, 0.04F, 0.04F);
    std::ostringstream text;
    text << "FINAL REPLAY (red)  t=" << std::fixed << std::setprecision(2)
         << state.time << "/" << result_.total_time << " s  speed="
         << state.velocity.norm() << " m/s\nsolve wall time="
         << result_.total_solve_time << " s  converged="
         << (result_.converged ? "yes" : "NO") << "  update="
         << std::scientific << std::setprecision(1) << result_.maximum_update;
    status.text = text.str();
    array->markers.push_back(std::move(status));
  }

  void publishMarkers(visualization_msgs::msg::MarkerArray array)
  {
    marker_publisher_->publish(array);
    ++published_frames_;
    if (published_frames_ == 1U) {
      RCLCPP_INFO(
        get_logger(), "Published first atomic RViz frame with %zu markers in frame '%s'",
        array.markers.size(), frame_id_.c_str());
    } else if (published_frames_ == 5U &&
      marker_publisher_->get_subscription_count() == 0U)
    {
      RCLCPP_WARN(
        get_logger(), "No RViz MarkerArray subscriber on %s",
        marker_publisher_->get_topic_name());
    } else if (!subscriber_reported_ &&
      marker_publisher_->get_subscription_count() > 0U)
    {
      subscriber_reported_ = true;
      RCLCPP_INFO(get_logger(), "RViz MarkerArray subscriber connected");
    }
  }

  void publishLiveOptimizationFrame(const OmTrajectoryIteration &iteration)
  {
    if (iteration.states.empty()) {
      visualization_msgs::msg::MarkerArray array;
      appendWaypoints(&array);
      auto status = baseMarker(
        0, "status", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      status.pose.position.z = 1.0;
      status.scale.z = 0.18;
      status.color = color(0.8F, 0.0F, 0.0F);
      status.text = iteration.solver_status;
      array.markers.push_back(std::move(status));
      publishMarkers(std::move(array));
      return;
    }
    visualization_msgs::msg::MarkerArray array;
    appendWaypoints(&array);
    appendTrajectoryAndThrust(iteration.states, color(1.0F, 0.42F, 0.02F), &array);
    appendOptimizationStatus(iteration, &array);
    publishMarkers(std::move(array));
  }

  void publishVehicleFrame(double elapsed_seconds)
  {
    visualization_msgs::msg::MarkerArray array;
    appendWaypoints(&array);
    if (!result_.success || result_.states.empty() || result_.total_time <= 0.0) {
      auto status = baseMarker(
        0, "status", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      status.pose.position.z = 1.0;
      status.scale.z = 0.18;
      status.color = color(0.8F, 0.0F, 0.0F);
      status.text = "No final trajectory: " + result_.status;
      array.markers.push_back(std::move(status));
      publishMarkers(std::move(array));
      return;
    }

    flight_time_ += elapsed_seconds * vehicle_playback_speed_;
    if (flight_time_ >= result_.total_time) {
      flight_time_ = std::fmod(flight_time_, result_.total_time);
    }
    const auto vehicle_state = OmTrajectoryOptimizer::sample(result_, flight_time_);
    appendTrajectoryAndThrust(
      final_display_states_.empty() ? result_.states : final_display_states_,
      color(0.95F, 0.02F, 0.02F), &array);
    appendVehicle(vehicle_state, &array);
    publishMarkers(std::move(array));
  }

  void publishFrame()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const double elapsed_seconds = std::chrono::duration<double>(
      steady_now - last_timer_time_).count();
    last_timer_time_ = steady_now;

    publishVehicleFrame(std::max(0.0, elapsed_seconds));
  }

  std::string frame_id_;
  std::string marker_topic_;
  double vehicle_publish_rate_hz_{30.0};
  double vehicle_playback_speed_{1.0};
  int final_display_samples_{240};
  int thrust_arrow_stride_{1};
  double thrust_arrow_scale_{0.03};
  double trajectory_width_{0.055};
  bool save_trajectory_{true};
  std::string trajectory_output_file_;
  std::size_t published_frames_{0};
  std::size_t live_progress_count_{0};
  double flight_time_{0.0};
  bool subscriber_reported_{false};
  std::chrono::steady_clock::time_point last_timer_time_;
  std::vector<OmTrajectoryWaypoint, Eigen::aligned_allocator<OmTrajectoryWaypoint>> waypoints_;
  std::vector<OmTrajectoryState, Eigen::aligned_allocator<OmTrajectoryState>>
    final_display_states_;
  OmTrajectoryResult result_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OmTrajectoryVisualizer>();
    if (node->get_parameter("exit_after_solve").as_bool()) {
      const int code = node->succeeded() ? 0 : 1; rclcpp::shutdown(); return code;
    }
    rclcpp::spin(node);
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("omtraj_visualizer"), "%s", exception.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
