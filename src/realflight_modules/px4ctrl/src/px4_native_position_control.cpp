#include <px4ctrl/px4_native_position_control.h>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;

Px4NativePositionControl::Px4NativePositionControl(const rclcpp::NodeOptions &options)
: Node("px4_native_position_node", options),
  reached_since_(get_clock()->now()), rc_stamp_(get_clock()->now()),
  local_position_stamp_(get_clock()->now()), attitude_stamp_(get_clock()->now()),
  state_enter_time_(get_clock()->now()), last_loop_time_(get_clock()->now()),
  last_command_request_(get_clock()->now() - rclcpp::Duration::from_seconds(1.0))
{
  loadMissionParameters();

  rmw_qos_profile_t profile = rmw_qos_profile_sensor_data;
  profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  const auto px4_qos = rclcpp::QoS(rclcpp::QoSInitialization(profile.history, 5), profile);

  offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);
  trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);
  // 同时监听实机与仿真 RC；首个有效来源会锁定，防止两路挡位相互覆盖。
  manual_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
    "/fmu/out/manual_control_setpoint", px4_qos,
    std::bind(&Px4NativePositionControl::manualControlCallback, this, std::placeholders::_1));
  joystick_sub_ = create_subscription<joy_msgs::msg::JoyStick>(
    "/joyStick", 10,
    std::bind(&Px4NativePositionControl::simulationJoystickCallback, this, std::placeholders::_1));
  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1", px4_qos,
    std::bind(&Px4NativePositionControl::statusCallback, this, std::placeholders::_1));
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", px4_qos,
    std::bind(&Px4NativePositionControl::localPositionCallback, this, std::placeholders::_1));
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
    "/fmu/out/vehicle_attitude", px4_qos,
    std::bind(&Px4NativePositionControl::attitudeCallback, this, std::placeholders::_1));
  command_client_ = create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
  timer_ = create_wall_timer(50ms, std::bind(&Px4NativePositionControl::timerCallback, this));

  if (simulation_enabled_) {
    // 仿真模式不等待遥控挡位；反馈就绪后由 timerCallback 自动启动完整任务流程。
    aux1_ = Gear::Up;
    start_requested_ = true;
  }

  RCLCPP_INFO(get_logger(),
    "PX4 waypoint mission ready: %zu NED points, speed control=%s, speed=%.2f m/s, simulation=%s",
    waypoints_ned_.size(), speed_control_enabled_ ? "on" : "off", flight_speed_,
    simulation_enabled_ ? "on" : "off");
}

void Px4NativePositionControl::loadMissionParameters()
{
  configured_point_count_ = declare_parameter<int>("mission.point_count", 1);
  const auto flat_points = declare_parameter<std::vector<double>>(
    "mission.points_ned", {0.0, 0.0, -1.0});
  speed_control_enabled_ = declare_parameter<bool>("mission.speed_control.enabled", false);
  flight_speed_ = declare_parameter<double>("mission.speed_control.speed", 1.0);
  acceptance_radius_ = declare_parameter<double>("mission.acceptance_radius", 0.20);
  waypoint_hold_time_ = declare_parameter<double>("mission.waypoint_hold_time", 0.30);
  prestream_duration_ = declare_parameter<double>("mission.prestream_duration", 1.0);
  simulation_enabled_ = declare_parameter<bool>("simulation.enabled", false);

  mission_config_valid_ = configured_point_count_ > 0 &&
    flat_points.size() == static_cast<std::size_t>(3 * configured_point_count_) &&
    std::all_of(flat_points.begin(), flat_points.end(), [](double value) {
      return std::isfinite(value);
    }) && flight_speed_ > 0.0 && acceptance_radius_ > 0.0 &&
    waypoint_hold_time_ >= 0.0 && prestream_duration_ >= 1.0;
  if (!mission_config_valid_) {
    RCLCPP_FATAL(get_logger(),
      "Invalid mission config: point_count=%d requires exactly %d finite values; "
      "speed/radius must be positive and prestream_duration >= 1.0 s",
      configured_point_count_, 3 * configured_point_count_);
    throw std::runtime_error("invalid PX4 native waypoint mission configuration");
  }

  waypoints_ned_.reserve(static_cast<std::size_t>(configured_point_count_));
  for (int i = 0; i < configured_point_count_; ++i) {
    waypoints_ned_.emplace_back(
      flat_points[3 * i], flat_points[3 * i + 1], flat_points[3 * i + 2]);
  }
}

Px4NativePositionControl::Gear Px4NativePositionControl::decodeGear(
  float value, Gear previous)
{
  if (!std::isfinite(value)) return previous;
  if (value > 0.5F) return Gear::Up;
  if (value < -0.5F) return Gear::Down;
  return Gear::Mid;
}

void Px4NativePositionControl::updateRc(RcSource source, float aux1, bool valid)
{
  if (!valid || !std::isfinite(aux1)) return;
  const auto now = get_clock()->now();
  if (active_rc_source_ != RcSource::None && active_rc_source_ != source &&
      (now - rc_stamp_).seconds() < 1.0) return;
  active_rc_source_ = source;
  const Gear old_aux1 = aux1_;
  aux1_ = decodeGear(aux1, aux1_);
  rc_stamp_ = now;

  if (aux1_ == Gear::Up && old_aux1 != Gear::Up) handleAux1Up();
  if (aux1_ == Gear::Down && old_aux1 != Gear::Down) handleAux1Down();
}

void Px4NativePositionControl::handleAux1Up()
{
  const auto now = get_clock()->now();
  start_requested_ = true;
  if (!mission_config_valid_ || estimator_reset_latched_) {
    RCLCPP_ERROR(get_logger(), "Mission start rejected: invalid config or estimator reset latch");
    start_requested_ = false;
    return;
  }
  if (!feedbackReady(now)) {
    // UP 请求保持锁存；反馈随后变为有效时 timerCallback 会自动继续，不要求重拨开关。
    RCLCPP_WARN(get_logger(), "Mission start pending: waiting for local position/attitude");
    return;
  }

  if (mission_state_ == MissionState::Hover && offboard_ever_entered_) {
    // DOWN 后再次 UP：从当前未完成航点继续，限速参考从当前悬停点重新起步。
    position_setpoint_ned_ = current_position_ned_;
    reached_timer_running_ = false;
    mission_state_ = MissionState::Flying;
    start_requested_ = false;
    state_enter_time_ = now;
    RCLCPP_INFO(get_logger(), "Mission resumed at waypoint %zu", waypoint_index_ + 1U);
    return;
  }

  waypoint_index_ = 0U;
  // PX4 官方 Offboard 示例在切换模式和解锁之前，就连续发送最终起飞位置目标。
  // 不能一直预发送“当前位置”并在解锁后才给第一点，否则 PX4 可能始终判断为
  // landed，并在看到明确起飞意图之前触发 preflight auto-disarm。
  position_setpoint_ned_ = waypoints_ned_.front();
  hold_yaw_ned_ = current_yaw_ned_;
  reached_timer_running_ = false;
  offboard_ever_entered_ = false;
  mission_completed_ = false;
  mission_state_ = MissionState::Prestream;
  start_requested_ = false;
  state_enter_time_ = now;
  RCLCPP_INFO(get_logger(),
    "Mission requested: prestream first waypoint [%.2f, %.2f, %.2f] before OFFBOARD/arm",
    position_setpoint_ned_.x(), position_setpoint_ned_.y(), position_setpoint_ned_.z());
}

void Px4NativePositionControl::handleAux1Down()
{
  start_requested_ = false;
  if (mission_state_ == MissionState::Idle) return;
  if (!offboard_ever_entered_ &&
      nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
    // 尚未进入 OFFBOARD/解锁时，DOWN 只取消启动流程。
    mission_state_ = MissionState::Idle;
    RCLCPP_INFO(get_logger(), "Mission start cancelled before OFFBOARD");
    return;
  }
  // 已进入任务后，DOWN 不降落、不切 MANUAL：锁定拨杆瞬间的当前 NED 位置持续悬停。
  captureHoverPoint();
  mission_state_ = MissionState::Hover;
  state_enter_time_ = get_clock()->now();
  RCLCPP_WARN(get_logger(), "Mission paused by AUX1 DOWN; holding current position");
}

void Px4NativePositionControl::manualControlCallback(
  const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
{
  if (simulation_enabled_) return;
  updateRc(RcSource::Px4, msg->aux1, msg->valid);
}

void Px4NativePositionControl::simulationJoystickCallback(
  const joy_msgs::msg::JoyStick::SharedPtr msg)
{
  if (simulation_enabled_) return;
  updateRc(RcSource::Simulation, msg->aux1, true);
}

void Px4NativePositionControl::statusCallback(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  nav_state_ = msg->nav_state;
  arming_state_ = msg->arming_state;
  preflight_checks_pass_ = msg->pre_flight_checks_pass;
}

void Px4NativePositionControl::localPositionCallback(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  const bool reset = position_counters_initialized_ &&
    (xy_reset_counter_ != msg->xy_reset_counter || z_reset_counter_ != msg->z_reset_counter);
  xy_reset_counter_ = msg->xy_reset_counter;
  z_reset_counter_ = msg->z_reset_counter;
  position_counters_initialized_ = true;
  current_position_ned_ << msg->x, msg->y, msg->z;
  local_position_stamp_ = get_clock()->now();
  local_position_valid_ = msg->xy_valid && msg->z_valid && current_position_ned_.allFinite();

  if (reset) {
    // 配置中的绝对本地 NED 航点在 EKF 原点重置后失去原物理含义，禁止自动恢复任务。
    estimator_reset_latched_ = true;
    if (local_position_valid_) captureHoverPoint();
    mission_state_ = offboard_ever_entered_ ? MissionState::Hover : MissionState::Idle;
    RCLCPP_ERROR(get_logger(),
      "PX4 local-position reset: mission latched; restart node after checking coordinates");
  }
}

void Px4NativePositionControl::attitudeCallback(
  const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  const double w = msg->q[0];
  const double x = msg->q[1];
  const double y = msg->q[2];
  const double z = msg->q[3];
  const double norm_sq = w*w + x*x + y*y + z*z;
  attitude_valid_ = std::isfinite(norm_sq) && norm_sq > 1.0e-12;
  if (!attitude_valid_) return;

  // PX4 q 是 FRD->NED；直接提取 NED yaw，不经过项目主控制链的 NWU 转换。
  current_yaw_ned_ = static_cast<float>(std::atan2(
    2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)));
  if (attitude_counter_initialized_ && quat_reset_counter_ != msg->quat_reset_counter) {
    hold_yaw_ned_ = current_yaw_ned_;
    RCLCPP_WARN(get_logger(), "PX4 attitude reset: yaw hold realigned");
  }
  quat_reset_counter_ = msg->quat_reset_counter;
  attitude_counter_initialized_ = true;
  attitude_stamp_ = get_clock()->now();
}

bool Px4NativePositionControl::rcFresh(const rclcpp::Time &now) const
{
  return active_rc_source_ != RcSource::None && (now - rc_stamp_).seconds() < 1.0;
}

bool Px4NativePositionControl::feedbackReady(const rclcpp::Time &now) const
{
  return local_position_valid_ && attitude_valid_ &&
    (now - local_position_stamp_).seconds() < 0.5 &&
    (now - attitude_stamp_).seconds() < 0.5;
}

void Px4NativePositionControl::captureHoverPoint()
{
  position_setpoint_ned_ = current_position_ned_;
  hold_yaw_ned_ = current_yaw_ned_;
  reached_timer_running_ = false;
}

void Px4NativePositionControl::updateMissionReference(double dt, const rclcpp::Time &now)
{
  if (waypoint_index_ >= waypoints_ned_.size()) {
    mission_state_ = MissionState::Hover;
    return;
  }
  const Eigen::Vector3d &waypoint = waypoints_ned_[waypoint_index_];
  if (speed_control_enabled_) {
    const Eigen::Vector3d delta = waypoint - position_setpoint_ned_;
    const double step = flight_speed_ * std::clamp(dt, 0.0, 0.10);
    if (delta.norm() <= step || delta.norm() < 1.0e-9) {
      position_setpoint_ned_ += delta;
    } else {
      position_setpoint_ned_ += delta.normalized() * step;
    }
  } else {
    // 关闭节点限速时直接把航点交给 PX4；实际速度由 PX4 参数限制。
    position_setpoint_ned_ = waypoint;
  }

  const bool inside = (current_position_ned_ - waypoint).norm() <= acceptance_radius_;
  if (!inside) {
    reached_timer_running_ = false;
    return;
  }
  if (!reached_timer_running_) {
    reached_timer_running_ = true;
    reached_since_ = now;
    return;
  }
  if ((now - reached_since_).seconds() < waypoint_hold_time_) return;

  RCLCPP_INFO(get_logger(), "Waypoint %zu/%zu reached", waypoint_index_ + 1U,
    waypoints_ned_.size());
  ++waypoint_index_;
  reached_timer_running_ = false;
  if (waypoint_index_ >= waypoints_ned_.size()) {
    // 完成任务后保持最后一个航点，不自动降落。
    position_setpoint_ned_ = waypoints_ned_.back();
    mission_completed_ = true;
    mission_state_ = MissionState::Hover;
    RCLCPP_INFO(get_logger(), "Mission complete; holding final waypoint");
  }
}

void Px4NativePositionControl::publishOffboardHeartbeat()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = true;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  offboard_mode_pub_->publish(msg);
}

void Px4NativePositionControl::publishTrajectorySetpoint()
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  msg.position = {static_cast<float>(position_setpoint_ned_.x()),
    static_cast<float>(position_setpoint_ned_.y()),
    static_cast<float>(position_setpoint_ned_.z())};
  msg.velocity = {nan, nan, nan};
  msg.acceleration = {nan, nan, nan};
  msg.jerk = {nan, nan, nan};
  msg.yaw = hold_yaw_ned_;
  msg.yawspeed = nan;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_pub_->publish(msg);
}

void Px4NativePositionControl::requestCommand(
  PendingCommand pending, uint16_t command, float param1, float param2)
{
  const auto now = get_clock()->now();
  // 不同命令不共享 0.5 s 冷却：OFFBOARD 获得确认后应立即发送 arm，接近 PX4
  // 官方示例“预发送完成后紧接着切模式并解锁”的时序。同一种命令仍限频重试。
  if (command_in_flight_ || !command_client_->service_is_ready() ||
      (pending == last_requested_command_ &&
       (now - last_command_request_).seconds() < 0.5)) return;
  auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
  request->request.command = command;
  request->request.param1 = param1;
  request->request.param2 = param2;
  request->request.target_system = 1;
  request->request.target_component = 1;
  request->request.source_system = 1;
  request->request.source_component = 1;
  request->request.from_external = true;
  request->request.timestamp = now.nanoseconds() / 1000;
  command_in_flight_ = true;
  last_requested_command_ = pending;
  last_command_request_ = now;
  command_client_->async_send_request(request,
    [this, pending](rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
      commandResponse(pending, future);
    });
}

void Px4NativePositionControl::commandResponse(
  PendingCommand pending, rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
{
  command_in_flight_ = false;
  const uint8_t result = future.get()->reply.result;
  if (result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
    RCLCPP_INFO(get_logger(), "PX4 accepted command type %d", static_cast<int>(pending));
  } else {
    RCLCPP_ERROR(get_logger(), "PX4 rejected command type %d, result=%u",
      static_cast<int>(pending), result);
  }
}

void Px4NativePositionControl::timerCallback()
{
  const auto now = get_clock()->now();
  const double dt = (now - last_loop_time_).seconds();
  last_loop_time_ = now;

  if (!simulation_enabled_ && !rcFresh(now)) {
    // 已经进入 OFFBOARD 时尽量保持当前位置；反馈也失效才停止流并让 PX4 failsafe 接管。
    if (offboard_ever_entered_ && feedbackReady(now)) {
      if (mission_state_ != MissionState::Hover) captureHoverPoint();
      mission_state_ = MissionState::Hover;
    } else {
      return;
    }
  }
  if (mission_state_ == MissionState::Idle && start_requested_ &&
      (simulation_enabled_ || aux1_ == Gear::Up) &&
      feedbackReady(now)) {
    handleAux1Up();
  }
  if (mission_state_ == MissionState::Idle) return;

  // PX4 官方 Offboard 时序要求 proof-of-life 连续存在。心跳不能依赖由 PX4
  // 回传给 ROS 的位置/姿态是否及时，否则一次短暂的输出话题延迟就会让本节点
  // 主动断掉输入心跳，PX4 随后必然触发 "No offboard signal"。
  publishOffboardHeartbeat();
  publishTrajectorySetpoint();

  if (!feedbackReady(now)) {
    if (!feedback_loss_active_) {
      feedback_loss_active_ = true;
      if (mission_state_ == MissionState::Prestream) {
        // 恢复后重新累计完整预发送时间，不使用反馈中断期间经过的时间。
        state_enter_time_ = now;
      }
      if (offboard_ever_entered_ &&
          arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
          local_position_valid_ && current_position_ned_.allFinite()) {
        // ROS 反馈链路异常时冻结在最后收到的位置，不继续推进航点；PX4 若自身
        // 估计也无效，仍会由飞控内部的位置失效保护决定是否退出 OFFBOARD。
        captureHoverPoint();
        mission_state_ = MissionState::Hover;
      }
    }
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "Position/attitude feedback stale; OFFBOARD heartbeat continues, mission progression frozen");
    return;
  }
  if (feedback_loss_active_) {
    feedback_loss_active_ = false;
    RCLCPP_WARN(get_logger(),
      "Position/attitude feedback recovered; OFFBOARD heartbeat was maintained");
  }

  if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
    offboard_ever_entered_ = true;
  } else if (offboard_ever_entered_) {
    if (mission_completed_) {
      // 全部航点已经完成后，PX4 离开 OFFBOARD 不再归类为任务中断故障。
      // 节点转入 Idle 并停止发送，不自动重新进入模式或再次解锁。
      offboard_ever_entered_ = false;
      mission_state_ = MissionState::Idle;
      RCLCPP_INFO(get_logger(),
        "PX4 left OFFBOARD after mission completion; mission controller stopped normally");
      return;
    }
    // 飞行途中退出仍属于安全故障：不抢回模式，必须重启节点并审核原因。
    estimator_reset_latched_ = true;
    mission_state_ = MissionState::Idle;
    RCLCPP_ERROR(get_logger(), "PX4 left OFFBOARD unexpectedly; automatic re-entry inhibited");
    return;
  }

  switch (mission_state_) {
    case MissionState::Prestream:
      if ((now - state_enter_time_).seconds() >= prestream_duration_) {
        mission_state_ = MissionState::WaitForOffboard;
      }
      break;
    case MissionState::WaitForOffboard:
      if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        mission_state_ = MissionState::WaitForArm;
      } else {
        requestCommand(PendingCommand::EnterOffboard,
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      }
      break;
    case MissionState::WaitForArm:
      if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
        // 第一航点在整个预发送、切模式和解锁阶段已经持续发布；此处绝不能
        // 重置回当前位置，否则会撤销 PX4 刚收到的起飞意图。
        mission_state_ = MissionState::Flying;
        RCLCPP_INFO(get_logger(),
          "Vehicle armed in OFFBOARD; first waypoint already active [%.2f, %.2f, %.2f]",
          position_setpoint_ned_.x(), position_setpoint_ned_.y(), position_setpoint_ned_.z());
      } else if (preflight_checks_pass_) {
        requestCommand(PendingCommand::Arm,
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for PX4 preflight checks before arming");
      }
      break;
    case MissionState::Flying:
      updateMissionReference(dt, now);
      break;
    case MissionState::Hover:
    case MissionState::Idle:
      break;
  }
}
