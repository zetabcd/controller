#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4debug_msgs/msg/px4ctrl_debug.hpp>
#include <px4debug_msgs/msg/px4ratectrl_debug.hpp>
#include <ratectrl_msgs/msg/rates_thrust_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace
{

template<typename MessageT>
struct Latest
{
  typename MessageT::SharedPtr message;
  rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
};

std::string defaultLogDirectory()
{
  if (const char * ros_home = std::getenv("ROS_HOME")) {
    return (std::filesystem::path(ros_home) / "flight_logs").string();
  }
  if (const char * home = std::getenv("HOME")) {
    return (std::filesystem::path(home) / ".ros" / "flight_logs").string();
  }
  return "flight_logs";
}

std::string timestampedFilename()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &time);
#else
  localtime_r(&time, &local_time);
#endif
  std::ostringstream name;
  name << "flight_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".csv";
  return name.str();
}

double ageSeconds(const rclcpp::Time & now, const rclcpp::Time & received_at)
{
  if (received_at.nanoseconds() == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(0.0, (now - received_at).seconds());
}

template<typename ValueT>
void append(std::ostringstream & row, const ValueT & value)
{
  row << ',' << value;
}

void appendBool(std::ostringstream & row, bool value)
{
  row << ',' << (value ? 1 : 0);
}

}  // namespace

class FlightDataRecorder final : public rclcpp::Node
{
public:
  FlightDataRecorder()
  : Node("flight_data_recorder")
  {
    std::string output_directory = declare_parameter<std::string>(
      "output_directory", defaultLogDirectory());
    if (output_directory.empty()) {
      output_directory = defaultLogDirectory();
    }
    const double record_rate_hz = std::clamp(
      declare_parameter<double>("record_rate_hz", 50.0), 1.0, 500.0);
    flush_interval_rows_ = static_cast<std::size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("flush_interval_rows", 50)));
    max_pending_rows_ = static_cast<std::size_t>(std::max<int64_t>(
      100, declare_parameter<int64_t>("max_pending_rows", 10000)));
    record_only_when_ready_ = declare_parameter<bool>("record_only_when_ready", true);

    std::filesystem::create_directories(output_directory);
    output_path_ = std::filesystem::path(output_directory) / timestampedFilename();
    stream_.open(output_path_, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) {
      throw std::runtime_error("cannot open flight log: " + output_path_.string());
    }
    stream_ << header() << '\n';
    stream_.flush();

    rmw_qos_profile_t profile = rmw_qos_profile_sensor_data;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    const auto sensor_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(profile.history, 10), profile);

    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", sensor_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        update(position_, std::move(msg));
      });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", sensor_qos,
      [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        update(attitude_, std::move(msg));
      });
    sensor_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
      "/fmu/out/sensor_combined", sensor_qos,
      [this](px4_msgs::msg::SensorCombined::SharedPtr msg) {
        update(sensor_, std::move(msg));
      });
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", sensor_qos,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        update(status_, std::move(msg));
      });
    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", sensor_qos,
      [this](px4_msgs::msg::BatteryStatus::SharedPtr msg) {
        update(battery_, std::move(msg));
      });
    actuator_sub_ = create_subscription<px4_msgs::msg::ActuatorMotors>(
      "/fmu/in/actuator_motors", sensor_qos,
      [this](px4_msgs::msg::ActuatorMotors::SharedPtr msg) {
        update(actuator_, std::move(msg));
      });
    setpoint_sub_ = create_subscription<ratectrl_msgs::msg::RatesThrustSetpoint>(
      "/rates_thrust_setpoint", sensor_qos,
      [this](ratectrl_msgs::msg::RatesThrustSetpoint::SharedPtr msg) {
        update(setpoint_, std::move(msg));
      });
    control_debug_sub_ = create_subscription<px4debug_msgs::msg::Px4ctrlDebug>(
      "/debugPx4/ctrl", rclcpp::QoS(10),
      [this](px4debug_msgs::msg::Px4ctrlDebug::SharedPtr msg) {
        update(control_debug_, std::move(msg));
      });
    rate_debug_sub_ = create_subscription<px4debug_msgs::msg::Px4ratectrlDebug>(
      "/debugPx4/ratectrl", rclcpp::QoS(10),
      [this](px4debug_msgs::msg::Px4ratectrlDebug::SharedPtr msg) {
        update(rate_debug_, std::move(msg));
      });

    start_time_ = get_clock()->now();
    writer_thread_ = std::thread(&FlightDataRecorder::writerLoop, this);
    const auto period = std::chrono::duration<double>(1.0 / record_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FlightDataRecorder::recordSnapshot, this));
    RCLCPP_INFO(
      get_logger(), "Recording flight data at %.1f Hz to %s", record_rate_hz,
      output_path_.c_str());
  }

  ~FlightDataRecorder() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }
    queue_condition_.notify_one();
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }
    if (dropped_rows_ > 0) {
      RCLCPP_WARN(
        get_logger(), "Flight log closed with %zu dropped rows", dropped_rows_);
    }
  }

private:
  template<typename MessageT>
  void update(Latest<MessageT> & destination, typename MessageT::SharedPtr message)
  {
    destination.message = std::move(message);
    destination.received_at = get_clock()->now();
  }

  static const char * header()
  {
    return
      "ros_time_s,elapsed_s,state,nav_state,arming_state,"
      "position_age_s,attitude_age_s,imu_age_s,debug_age_s,"
      "actual_p_x,actual_p_y,actual_p_z,actual_v_x,actual_v_y,actual_v_z,"
      "actual_q_w,actual_q_x,actual_q_y,actual_q_z,"
      "gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,"
      "ref_p_x,ref_p_y,ref_p_z,ref_v_x,ref_v_y,ref_v_z,"
      "position_error_x,position_error_y,position_error_z,position_error_norm,"
      "velocity_error_x,velocity_error_y,velocity_error_z,velocity_error_norm,"
      "ref_a_x,ref_a_y,ref_a_z,des_q_w,des_q_x,des_q_y,des_q_z,"
      "des_rate_x,des_rate_y,des_rate_z,des_rate_dot_x,des_rate_dot_y,des_rate_dot_z,"
      "des_thrust_n,voltage_v,current_a,battery_remaining,"
      "rate_dot_x,rate_dot_y,rate_dot_z,tau_x,tau_y,tau_z,rate_solve_time_ms,"
      "motor_rpm_1,motor_rpm_2,motor_rpm_3,motor_rpm_4,"
      "motor_thrust_1,motor_thrust_2,motor_thrust_3,motor_thrust_4,"
      "actuator_1,actuator_2,actuator_3,actuator_4,"
      "position_valid,velocity_valid,source_position_timestamp_us,source_debug_timestamp_us";
  }

  void recordSnapshot()
  {
    if (record_only_when_ready_ && (!position_.message || !control_debug_.message)) {
      return;
    }

    const auto now = get_clock()->now();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto & p = position_.message;
    const auto & q = attitude_.message;
    const auto & imu = sensor_.message;
    const auto & dbg = control_debug_.message;
    const auto & rate = rate_debug_.message;
    const auto & sp = setpoint_.message;
    const auto & bat = battery_.message;

    // CSV统一使用控制器坐标：x前、y左、z上；PX4原始NED/FRD的y、z在此取反。
    const double px = p ? p->x : nan;
    const double py = p ? -p->y : nan;
    const double pz = p ? -p->z : nan;
    const double vx = p ? p->vx : nan;
    const double vy = p ? -p->vy : nan;
    const double vz = p ? -p->vz : nan;
    // debug消息为历史显示约定（NED/FRD符号），同样转换回控制器坐标。
    const double rpx = dbg ? dbg->ref_p_x : nan;
    const double rpy = dbg ? -dbg->ref_p_y : nan;
    const double rpz = dbg ? -dbg->ref_p_z : nan;
    const double rvx = dbg ? dbg->ref_v_x : nan;
    const double rvy = dbg ? -dbg->ref_v_y : nan;
    const double rvz = dbg ? -dbg->ref_v_z : nan;
    const double ex = px - rpx;
    const double ey = py - rpy;
    const double ez = pz - rpz;
    const double evx = vx - rvx;
    const double evy = vy - rvy;
    const double evz = vz - rvz;

    std::ostringstream row;
    row << std::setprecision(10) << now.seconds();
    append(row, (now - start_time_).seconds());
    append(row, dbg ? dbg->state : 0U);
    append(row, status_.message ? static_cast<unsigned>(status_.message->nav_state) : 0U);
    append(row, status_.message ? static_cast<unsigned>(status_.message->arming_state) : 0U);
    append(row, ageSeconds(now, position_.received_at));
    append(row, ageSeconds(now, attitude_.received_at));
    append(row, ageSeconds(now, sensor_.received_at));
    append(row, ageSeconds(now, control_debug_.received_at));
    append(row, px); append(row, py); append(row, pz);
    append(row, vx); append(row, vy); append(row, vz);
    append(row, q ? q->q[0] : nan);
    append(row, q ? q->q[1] : nan);
    append(row, q ? -q->q[2] : nan);
    append(row, q ? -q->q[3] : nan);
    append(row, imu ? imu->gyro_rad[0] : nan);
    append(row, imu ? -imu->gyro_rad[1] : nan);
    append(row, imu ? -imu->gyro_rad[2] : nan);
    append(row, imu ? imu->accelerometer_m_s2[0] : nan);
    append(row, imu ? -imu->accelerometer_m_s2[1] : nan);
    append(row, imu ? -imu->accelerometer_m_s2[2] : nan);
    append(row, rpx); append(row, rpy); append(row, rpz);
    append(row, rvx); append(row, rvy); append(row, rvz);
    append(row, ex); append(row, ey); append(row, ez);
    append(row, std::sqrt(ex * ex + ey * ey + ez * ez));
    append(row, evx); append(row, evy); append(row, evz);
    append(row, std::sqrt(evx * evx + evy * evy + evz * evz));
    append(row, dbg ? dbg->ref_a_x : nan);
    append(row, dbg ? -dbg->ref_a_y : nan);
    append(row, dbg ? -dbg->ref_a_z : nan);
    append(row, dbg ? dbg->des_q_w : nan);
    append(row, dbg ? dbg->des_q_x : nan);
    append(row, dbg ? -dbg->des_q_y : nan);
    append(row, dbg ? -dbg->des_q_z : nan);
    append(row, sp ? sp->bodyrates[0] : (dbg ? dbg->des_rate_x : nan));
    append(row, sp ? sp->bodyrates[1] : (dbg ? -dbg->des_rate_y : nan));
    append(row, sp ? sp->bodyrates[2] : (dbg ? -dbg->des_rate_z : nan));
    append(row, sp ? sp->rate_dot_ref[0] : nan);
    append(row, sp ? sp->rate_dot_ref[1] : nan);
    append(row, sp ? sp->rate_dot_ref[2] : nan);
    append(row, sp ? sp->thrust : (dbg ? dbg->des_thrust : nan));
    append(row, bat ? bat->voltage_v : (dbg ? dbg->voltage : nan));
    append(row, bat ? bat->current_a : nan);
    append(row, bat ? bat->remaining : nan);
    append(row, rate ? rate->cur_rate_dot_x : nan);
    append(row, rate ? -rate->cur_rate_dot_y : nan);
    append(row, rate ? -rate->cur_rate_dot_z : nan);
    append(row, rate ? rate->cur_tau_x : nan);
    append(row, rate ? -rate->cur_tau_y : nan);
    append(row, rate ? -rate->cur_tau_z : nan);
    append(row, rate ? rate->solve_time_ms : nan);
    for (std::size_t i = 0; i < 4; ++i) append(row, rate ? rate->des_motor_rpm[i] : nan);
    append(row, rate ? rate->des_u_1 : nan); append(row, rate ? rate->des_u_2 : nan);
    append(row, rate ? rate->des_u_3 : nan); append(row, rate ? rate->des_u_4 : nan);
    for (std::size_t i = 0; i < 4; ++i) {
      append(row, actuator_.message ? actuator_.message->control[i] : nan);
    }
    appendBool(row, p && p->xy_valid && p->z_valid);
    appendBool(row, p && p->v_xy_valid && p->v_z_valid);
    append(row, p ? p->timestamp : 0ULL);
    append(row, dbg ? dbg->timestamp : 0ULL);

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (pending_rows_.size() >= max_pending_rows_) {
        pending_rows_.pop_front();
        ++dropped_rows_;
      }
      pending_rows_.push_back(row.str());
    }
    queue_condition_.notify_one();
  }

  void writerLoop()
  {
    std::size_t rows_since_flush = 0;
    while (true) {
      std::deque<std::string> rows;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this] {return stopping_ || !pending_rows_.empty();});
        rows.swap(pending_rows_);
        if (rows.empty() && stopping_) {
          break;
        }
      }
      for (const auto & row : rows) {
        stream_ << row << '\n';
      }
      rows_since_flush += rows.size();
      if (rows_since_flush >= flush_interval_rows_) {
        stream_.flush();
        rows_since_flush = 0;
      }
      if (stopping_) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_rows_.empty()) {
          break;
        }
      }
    }
    stream_.flush();
    stream_.close();
  }

  Latest<px4_msgs::msg::VehicleLocalPosition> position_;
  Latest<px4_msgs::msg::VehicleAttitude> attitude_;
  Latest<px4_msgs::msg::SensorCombined> sensor_;
  Latest<px4_msgs::msg::VehicleStatus> status_;
  Latest<px4_msgs::msg::BatteryStatus> battery_;
  Latest<px4_msgs::msg::ActuatorMotors> actuator_;
  Latest<ratectrl_msgs::msg::RatesThrustSetpoint> setpoint_;
  Latest<px4debug_msgs::msg::Px4ctrlDebug> control_debug_;
  Latest<px4debug_msgs::msg::Px4ratectrlDebug> rate_debug_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_sub_;
  rclcpp::Subscription<ratectrl_msgs::msg::RatesThrustSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<px4debug_msgs::msg::Px4ctrlDebug>::SharedPtr control_debug_sub_;
  rclcpp::Subscription<px4debug_msgs::msg::Px4ratectrlDebug>::SharedPtr rate_debug_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  std::filesystem::path output_path_;
  std::ofstream stream_;
  std::thread writer_thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::deque<std::string> pending_rows_;
  std::size_t flush_interval_rows_{50};
  std::size_t max_pending_rows_{10000};
  std::size_t dropped_rows_{0};
  bool record_only_when_ready_{true};
  std::atomic<bool> stopping_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FlightDataRecorder>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("flight_data_recorder"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
