#include <px4debug_msgs/msg/px4ctrl_debug.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace
{

bool finitePoint(const geometry_msgs::msg::Point &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double squaredDistance(const geometry_msgs::msg::Point &lhs, const geometry_msgs::msg::Point &rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return dx * dx + dy * dy + dz * dz;
}

geometry_msgs::msg::Point projectNedToDisplay(double north, double east, double down)
{
  // 与 quadsim 红/绿轨迹的显示坐标完全一致：x 前、y 左、z 上。
  geometry_msgs::msg::Point point;
  point.x = north;
  point.y = -east;
  point.z = -down;
  return point;
}

geometry_msgs::msg::Quaternion projectPx4QuaternionToDisplay(
  double w, double x, double y, double z)
{
  // 与 quadsim 的 FRD/NED -> FLU 显示转换一致。
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = w;
  quaternion.x = x;
  quaternion.y = -y;
  quaternion.z = -z;
  return quaternion;
}

bool finiteQuaternion(const geometry_msgs::msg::Quaternion &quaternion)
{
  return std::isfinite(quaternion.w) && std::isfinite(quaternion.x) &&
    std::isfinite(quaternion.y) && std::isfinite(quaternion.z) &&
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z > 1.0e-12;
}

}  // namespace

class RealflightTrajectoryVisualizer final : public rclcpp::Node
{
public:
  RealflightTrajectoryVisualizer()
  : Node("realflight_trajectory_visualizer"),
    last_actual_sample_(get_clock()->now()),
    last_reference_sample_(get_clock()->now())
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const std::string debug_topic =
      declare_parameter<std::string>("debug_topic", "/debugPx4/ctrl");
    const std::string position_topic = declare_parameter<std::string>(
      "position_topic", "/fmu/out/vehicle_local_position");
    const std::string marker_topic = declare_parameter<std::string>(
      "marker_topic", "/realflight/trajectory_markers");
    max_points_ = static_cast<std::size_t>(std::max<int64_t>(
      100, declare_parameter<int64_t>("max_points", 3000)));
    sample_period_ = std::max(0.01, declare_parameter<double>("sample_period", 0.05));
    minimum_step_ = std::max(0.0, declare_parameter<double>("minimum_step", 0.005));
    line_width_ = std::max(0.005, declare_parameter<double>("line_width", 0.035));

    rmw_qos_profile_t px4_profile = rmw_qos_profile_sensor_data;
    px4_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    px4_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    const auto px4_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(px4_profile.history, 5), px4_profile);

    actual_subscription_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      position_topic, px4_qos,
      std::bind(&RealflightTrajectoryVisualizer::actualCallback, this, std::placeholders::_1));
    attitude_subscription_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", px4_qos,
      std::bind(&RealflightTrajectoryVisualizer::attitudeCallback, this, std::placeholders::_1));
    debug_subscription_ = create_subscription<px4debug_msgs::msg::Px4ctrlDebug>(
      debug_topic, rclcpp::QoS(10),
      std::bind(&RealflightTrajectoryVisualizer::debugCallback, this, std::placeholders::_1));

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, rclcpp::QoS(1).reliable().transient_local());
    publish_timer_ = create_wall_timer(
      50ms, std::bind(&RealflightTrajectoryVisualizer::publishMarkers, this));

    RCLCPP_INFO(
      get_logger(),
      "Real-flight trajectory view ready: actual=%s (green), reference=%s (red), markers=%s",
      position_topic.c_str(), debug_topic.c_str(), marker_topic.c_str());
  }

private:
  void actualCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
  {
    const bool estimator_reset = position_counters_initialized_ &&
      (xy_reset_counter_ != message->xy_reset_counter ||
      z_reset_counter_ != message->z_reset_counter);
    xy_reset_counter_ = message->xy_reset_counter;
    z_reset_counter_ = message->z_reset_counter;
    position_counters_initialized_ = true;
    if (estimator_reset) {
      // EKF 本地原点变化时不能用直线连接重置前后的两个坐标系。
      actual_history_.clear();
      last_actual_sample_ = get_clock()->now() -
        rclcpp::Duration::from_seconds(sample_period_);
    }
    actual_position_ = projectNedToDisplay(message->x, message->y, message->z);
    actual_valid_ = message->xy_valid && message->z_valid && finitePoint(actual_position_);
    actual_stamp_ = get_clock()->now();
    if (actual_valid_) {
      appendSample(actual_position_, actual_stamp_, &last_actual_sample_, &actual_history_);
    }
  }

  void debugCallback(const px4debug_msgs::msg::Px4ctrlDebug::SharedPtr message)
  {
    reference_position_ = projectNedToDisplay(
      message->ref_p_x, message->ref_p_y, message->ref_p_z);
    reference_valid_ = finitePoint(reference_position_);
    reference_stamp_ = get_clock()->now();
    controller_state_ = message->state;
    voltage_ = message->voltage;
    reference_attitude_ = projectPx4QuaternionToDisplay(
      message->des_q_w, message->des_q_x, message->des_q_y, message->des_q_z);
    reference_attitude_valid_ = finiteQuaternion(reference_attitude_);
    // 启动阶段 debug_msg 的参考位置通常是零。只在真正的位置控制状态记录红线，
    // 避免把默认原点和任务开始后的第一个参考点误连成一条“期望路径”。
    const bool trace_reference = controller_state_ == 2U || controller_state_ == 3U;
    if (trace_reference && !reference_trace_active_) {
      reference_history_.clear();
      last_reference_sample_ = reference_stamp_ - rclcpp::Duration::from_seconds(sample_period_);
    }
    reference_trace_active_ = trace_reference;
    if (reference_valid_ && trace_reference) {
      appendSample(
        reference_position_, reference_stamp_, &last_reference_sample_, &reference_history_);
    }
  }

  void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr message)
  {
    actual_attitude_ = projectPx4QuaternionToDisplay(
      message->q[0], message->q[1], message->q[2], message->q[3]);
    actual_attitude_valid_ = finiteQuaternion(actual_attitude_);
  }

  void appendSample(
    const geometry_msgs::msg::Point &point, const rclcpp::Time &now,
    rclcpp::Time *last_sample, std::deque<geometry_msgs::msg::Point> *history)
  {
    const bool enough_time = (now - *last_sample).seconds() >= sample_period_;
    const bool enough_motion = history->empty() ||
      squaredDistance(point, history->back()) >= minimum_step_ * minimum_step_;
    if (!history->empty() && (!enough_time || !enough_motion)) {
      return;
    }
    history->push_back(point);
    *last_sample = now;
    while (history->size() > max_points_) {
      history->pop_front();
    }
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

  visualization_msgs::msg::Marker trajectoryMarker(
    int id, const std::string &name_space,
    const std::deque<geometry_msgs::msg::Point> &history,
    float red, float green, float blue) const
  {
    auto marker = baseMarker(id, name_space, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.scale.x = line_width_;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    marker.points.assign(history.begin(), history.end());
    return marker;
  }

  visualization_msgs::msg::Marker vehicleMarker(
    int id, const std::string &name_space, const geometry_msgs::msg::Point &position,
    const geometry_msgs::msg::Quaternion &attitude,
    float red, float green, float blue) const
  {
    // 长方体的长轴是机体 x 轴，能同时看出偏航、俯仰和横滚。
    auto marker = baseMarker(id, name_space, visualization_msgs::msg::Marker::CUBE);
    marker.pose.position = position;
    marker.pose.orientation = attitude;
    marker.scale.x = 0.36;
    marker.scale.y = 0.22;
    marker.scale.z = 0.10;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    return marker;
  }

  const char *stateName(uint32_t state) const
  {
    switch (state) {
      case 0: return "MANUAL_ON";
      case 1: return "MANUAL";
      case 2: return "AUTO_HOVER";
      case 3: return "CMD";
      case 4: return "SAFE";
      case 5: return "ERROR";
      default: return "UNKNOWN";
    }
  }

  visualization_msgs::msg::Marker statusMarker(const rclcpp::Time &now) const
  {
    auto marker = baseMarker(0, "flight_status", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    marker.pose.position = actual_valid_ ? actual_position_ : reference_position_;
    marker.pose.position.z += 0.45;
    marker.scale.z = 0.18;
    marker.color.r = 0.1F;
    marker.color.g = 0.1F;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;

    const bool actual_fresh = actual_valid_ && (now - actual_stamp_).seconds() < 0.5;
    const bool reference_fresh = reference_valid_ && (now - reference_stamp_).seconds() < 0.5;
    const double error = actual_valid_ && reference_valid_ ?
      std::sqrt(squaredDistance(actual_position_, reference_position_)) : 0.0;
    std::ostringstream text;
    text << stateName(controller_state_) << "  error=" << std::fixed << std::setprecision(2)
         << error << " m  voltage=" << std::setprecision(1) << voltage_ << " V\n"
         << "actual:" << (actual_fresh ? "OK" : "STALE")
         << "  reference:" << (reference_fresh ? "OK" : "STALE");
    marker.text = text.str();
    return marker;
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.reserve(6);
    array.markers.push_back(trajectoryMarker(
      0, "actual_trajectory", actual_history_, 0.0F, 0.85F, 0.15F));
    array.markers.push_back(trajectoryMarker(
      0, "reference_trajectory", reference_history_, 0.95F, 0.05F, 0.05F));
    if (actual_valid_ && actual_attitude_valid_) {
      array.markers.push_back(vehicleMarker(
        0, "actual_vehicle", actual_position_, actual_attitude_, 0.0F, 0.85F, 0.15F));
    }
    if (reference_valid_ && reference_attitude_valid_) {
      array.markers.push_back(vehicleMarker(
        0, "reference_vehicle", reference_position_, reference_attitude_,
        0.95F, 0.05F, 0.05F));
    }
    if (actual_valid_ && reference_valid_) {
      auto error = baseMarker(0, "tracking_error", visualization_msgs::msg::Marker::LINE_LIST);
      error.scale.x = 0.012;
      error.color.r = 0.9F;
      error.color.g = 0.65F;
      error.color.b = 0.0F;
      error.color.a = 0.8F;
      error.points = {actual_position_, reference_position_};
      array.markers.push_back(std::move(error));
    }
    array.markers.push_back(statusMarker(get_clock()->now()));
    marker_publisher_->publish(array);
  }

  std::string frame_id_;
  std::size_t max_points_{3000};
  double sample_period_{0.05};
  double minimum_step_{0.005};
  double line_width_{0.035};

  geometry_msgs::msg::Point actual_position_;
  geometry_msgs::msg::Point reference_position_;
  geometry_msgs::msg::Quaternion actual_attitude_;
  geometry_msgs::msg::Quaternion reference_attitude_;
  std::deque<geometry_msgs::msg::Point> actual_history_;
  std::deque<geometry_msgs::msg::Point> reference_history_;
  bool actual_valid_{false};
  bool reference_valid_{false};
  bool actual_attitude_valid_{false};
  bool reference_attitude_valid_{false};
  bool reference_trace_active_{false};
  bool position_counters_initialized_{false};
  uint8_t xy_reset_counter_{0U};
  uint8_t z_reset_counter_{0U};
  uint32_t controller_state_{0};
  float voltage_{0.0F};
  rclcpp::Time actual_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reference_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_actual_sample_;
  rclcpp::Time last_reference_sample_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr actual_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
  rclcpp::Subscription<px4debug_msgs::msg::Px4ctrlDebug>::SharedPtr debug_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealflightTrajectoryVisualizer>());
  rclcpp::shutdown();
  return 0;
}
