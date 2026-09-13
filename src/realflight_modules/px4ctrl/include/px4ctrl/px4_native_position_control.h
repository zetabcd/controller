#ifndef PX4CTRL_PX4_NATIVE_POSITION_CONTROL_H_
#define PX4CTRL_PX4_NATIVE_POSITION_CONTROL_H_

#include <joy_msgs/msg/joy_stick.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <vector>

// 独立的 PX4 原生位置任务节点：只发送 NED 位置目标，位置环及以下控制全部由 PX4 完成。
// 本类不调用项目现有 PX4CtrlFSM、MPC、角速度环或电机分配。
class Px4NativePositionControl final : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Px4NativePositionControl(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  enum class Gear : int8_t {Down = -1, Mid = 0, Up = 1};
  enum class RcSource {None, Px4, Simulation};
  enum class MissionState {Idle, Prestream, WaitForOffboard, WaitForArm, Flying, Hover};
  enum class PendingCommand {None, EnterOffboard, Arm};

  void loadMissionParameters();
  void manualControlCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg);
  void simulationJoystickCallback(const joy_msgs::msg::JoyStick::SharedPtr msg);
  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void timerCallback();

  static Gear decodeGear(float value, Gear previous);
  void updateRc(RcSource source, float aux1, bool valid);
  void handleAux1Up();
  void handleAux1Down();
  bool rcFresh(const rclcpp::Time &now) const;
  bool feedbackReady(const rclcpp::Time &now) const;
  void captureHoverPoint();
  void updateMissionReference(double dt, const rclcpp::Time &now);
  void publishOffboardHeartbeat();
  void publishTrajectorySetpoint();
  void requestCommand(PendingCommand pending, uint16_t command, float param1, float param2 = 0.0F);
  void commandResponse(
    PendingCommand pending, rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_sub_;
  rclcpp::Subscription<joy_msgs::msg::JoyStick>::SharedPtr joystick_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> waypoints_ned_;
  Eigen::Vector3d current_position_ned_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_setpoint_ned_{Eigen::Vector3d::Zero()};
  std::size_t waypoint_index_{0U};
  int configured_point_count_{0};
  bool speed_control_enabled_{false};
  double flight_speed_{1.0};
  double acceptance_radius_{0.20};
  double waypoint_hold_time_{0.30};
  double prestream_duration_{1.0};
  bool simulation_enabled_{false};
  bool mission_config_valid_{false};
  bool reached_timer_running_{false};
  rclcpp::Time reached_since_;

  float current_yaw_ned_{0.0F};
  float hold_yaw_ned_{0.0F};
  bool local_position_valid_{false};
  bool attitude_valid_{false};
  bool preflight_checks_pass_{false};
  bool command_in_flight_{false};
  PendingCommand last_requested_command_{PendingCommand::None};
  bool start_requested_{false};
  bool offboard_ever_entered_{false};
  bool mission_completed_{false};
  bool estimator_reset_latched_{false};
  bool feedback_loss_active_{false};
  Gear aux1_{Gear::Down};
  RcSource active_rc_source_{RcSource::None};
  MissionState mission_state_{MissionState::Idle};
  uint8_t nav_state_{0U};
  uint8_t arming_state_{0U};
  bool position_counters_initialized_{false};
  bool attitude_counter_initialized_{false};
  uint8_t xy_reset_counter_{0U};
  uint8_t z_reset_counter_{0U};
  uint8_t quat_reset_counter_{0U};
  rclcpp::Time rc_stamp_;
  rclcpp::Time local_position_stamp_;
  rclcpp::Time attitude_stamp_;
  rclcpp::Time state_enter_time_;
  rclcpp::Time last_loop_time_;
  rclcpp::Time last_command_request_;
};

#endif  // PX4CTRL_PX4_NATIVE_POSITION_CONTROL_H_
