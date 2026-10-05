#pragma once
#include <px4ctrl/external_trajectory.h>
#include <gap_msgs/msg/execution_status.hpp>
#include <gap_msgs/msg/scene_status.hpp>
#include <gap_msgs/srv/upload_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <future>

namespace px4ctrl
{
// All ROS callbacks and poll run on the control node's single executor thread.
// Expensive validation owns an immutable copy and never accesses the FSM.
class ExternalExecution
{
public:
  ExternalExecution(rclcpp::Node &node, TrajectoryLimits limits, ExternalModel model,
    bool simulation);
  void poll(double now, bool command_mode, bool auto_hover, bool feedback_valid, const Eigen::Vector3d &p,
    const Eigen::Vector3d &v, const Eigen::Quaterniond &q,bool origin_valid,const Eigen::Vector3d &origin);
  bool active() const {return static_cast<bool>(active_);}
  double takeoffHeight() const {return takeoff_height_;}
  bool requestStart();
  bool ready(std::string &reason) const;
  std::shared_ptr<const Trajectory> activate(double now);
  bool finished(double now) const;
  void startClock(double now) {started_=now;}
  void complete();
  void stop();
private:
  struct Validation {std::shared_ptr<const Trajectory> trajectory; std::string reason; double elapsed_ms; uint64_t generation;};
  rclcpp::Node &node_;
  TrajectoryLimits limits_;
  ExternalModel model_;
  std::future<Validation> validation_;
  std::shared_ptr<const Trajectory> pending_, active_;
  std::string state_{"OUTSIDE_CMD"}, reason_{"Enter CMD to select a gap task"};
  std::string trajectory_id_, scene_id_, latest_scene_;
  double scene_received_{-1}, scene_stamp_{-1}, last_now_{-1}, started_{0}, publish_at_{0};
  bool scene_valid_{false}, hovering_{false}, feedback_valid_{false}, start_requested_{false};
  bool command_mode_{false};
  uint64_t generation_{0};
  double takeoff_height_{1.0};
  Eigen::Vector3d p_{Eigen::Vector3d::Zero()}, v_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  double position_tolerance_, speed_tolerance_, attitude_tolerance_, scene_timeout_;
  double validation_ms_{0};
  Eigen::Vector3d bounds_min_, bounds_max_;
  rclcpp::Service<gap_msgs::srv::UploadTrajectory>::SharedPtr upload_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_;
  rclcpp::Subscription<gap_msgs::msg::SceneStatus>::SharedPtr scene_;
  rclcpp::Publisher<gap_msgs::msg::ExecutionStatus>::SharedPtr status_;
};
}
