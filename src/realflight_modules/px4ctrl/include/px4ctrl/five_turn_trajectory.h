#pragma once

#include <px4ctrl/omtraj.h>

#include <Eigen/Geometry>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace px4ctrl
{

struct FiveTurnSample
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double time{0.0};
  double progress{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
  double thrust{0.0};
  bool valid{false};
};

struct DiscretizedFiveTurnTrajectory
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double requested_dt{0.0};
  double actual_dt{0.0};
  std::vector<FiveTurnSample, Eigen::aligned_allocator<FiveTurnSample>> points;
};

// 圆形外观与 BarrelRollHelixTrajectory 一致：入口直线、绕水平轴的圆形
// 翻滚段、出口直线。不同之处是全部五次多项式由论文式最小 jerk QP 求解。
struct FiveTurnHelixParameters
{
  Eigen::Vector3d start_position{0.0, 0.0, 1.0};
  Eigen::Vector3d roll_axis{1.0, 0.0, 0.0};
  int revolutions{5};
  int circle_waypoints_per_revolution{16};
  double radius{1.0};
  double axial_speed{0.5};
  double entry_duration{1.5};
  double roll_duration{9.0};
  double exit_duration{1.5};
  // 改进版QP不仅经过圆形位置路标，还软跟踪同一七阶相位种子产生的导数。
  // 这三个权重用于抑制上一版节点之间的加速度方向回绕和角速度尖峰。
  double velocity_tracking_weight{100.0};
  double acceleration_tracking_weight{10.0};
  double jerk_tracking_weight{0.1};
  double mass{1.5};
  double gravity{9.80665};
};

struct FiveTurnHelixValidationReport
{
  bool valid{false};
  double qp_equality_residual{0.0};
  double integrated_squared_jerk{0.0};
  double maximum_waypoint_error{0.0};
  double maximum_circle_seed_error{0.0};
  double maximum_attitude_constraint_error_deg{0.0};
  double maximum_connection_position_error{0.0};
  double maximum_connection_velocity_error{0.0};
  double maximum_connection_acceleration_error{0.0};
  double maximum_connection_jerk_error{0.0};
  double position_winding_angle_deg{0.0};
  double attitude_winding_angle_deg{0.0};
  double axial_advance{0.0};
  double minimum_thrust{0.0};
  double maximum_thrust{0.0};
  double maximum_speed{0.0};
  double maximum_acceleration{0.0};
  double maximum_jerk{0.0};
  double maximum_body_rate{0.0};
  double maximum_attitude_phase_backtracking_deg{0.0};
  double minimum_altitude{0.0};
  double maximum_altitude{0.0};
  std::vector<std::string> warnings;
};

// 每圈用多个圆周位置路标保证外观确实接近圆；0/90/180/270度节点的
// 加速度来自同一个七阶解析种子：其中两个分量对应论文的平行姿态约束，
// 第三个分量规定正推力幅值以消除方向反号。改进版还软跟踪种子的v/a/jerk，
// 区别于上一版“仅有硬路标”的QP。所有连接点仍保持 C3 连续。
class FiveTurnHelixTrajectory
{
public:
  bool generate(
    const FiveTurnHelixParameters & parameters,
    std::string * error = nullptr);

  FiveTurnSample sample(double time) const;
  DiscretizedFiveTurnTrajectory discretize(double requested_dt) const;
  FiveTurnHelixValidationReport validate(double check_dt = 0.001) const;

  // 仅用于灰色几何种子可视化和误差检查；控制器应使用 sample()/discretize()。
  Eigen::Vector3d analyticPosition(double time) const;

  bool generated() const {return generated_;}
  double duration() const {return total_duration_;}
  double rollStartTime() const {return parameters_.entry_duration;}
  double rollEndTime() const {return parameters_.entry_duration + parameters_.roll_duration;}
  int revolutions() const {return parameters_.revolutions;}
  int segmentCount() const {return segment_count_;}
  int rollSegmentCount() const {return roll_segment_count_;}
  double knotTime(int knot) const;
  Eigen::Vector3d waypoint(int knot) const;
  Eigen::Vector3d desiredThrustDirection(int knot) const;
  bool isAttitudeKnot(int knot) const;
  const Eigen::Vector3d & advanceAxis() const {return roll_axis_;}
  const Eigen::Vector3d & lateralAxis() const {return lateral_axis_;}
  const FiveTurnHelixParameters & parameters() const {return parameters_;}

private:
  struct SeedKinematics
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  };

  static constexpr int kDimension = 3;
  static constexpr int kCoefficientCount = 6;

  int coefficientIndex(int segment, int axis, int power) const;
  Eigen::Matrix<double, kCoefficientCount, 1> derivativeBasis(
    int derivative_order, double local_time, double duration) const;
  Eigen::Vector3d evaluateSegment(
    int segment, double local_time, int derivative_order) const;
  Eigen::Vector3d evaluate(double time, int derivative_order) const;
  int segmentForTime(double time) const;
  double segmentDuration(int segment) const;
  double rollPhaseFromLocalTime(double roll_local_time) const;
  double rollLocalTimeFromPhase(double phase) const;
  Eigen::Vector3d rollPosition(double roll_local_time, double phase) const;
  SeedKinematics analyticRollKinematics(double roll_local_time) const;

  FiveTurnHelixParameters parameters_;
  Eigen::Vector3d roll_axis_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d lateral_axis_{Eigen::Vector3d::UnitY()};
  Eigen::MatrixXd coefficients_;
  std::vector<double> knot_times_;
  std::vector<double> knot_phases_;
  int segment_count_{0};
  int roll_segment_count_{0};
  double total_duration_{0.0};
  double equality_residual_{0.0};
  double integrated_squared_jerk_{0.0};
  bool generated_{false};
};

// 当前项目的完整 CMD 轨迹参数：在论文五圈机动前增加平滑起飞和稳定段。
struct FiveTurnTrajectoryOptions
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d roll_axis{Eigen::Vector3d::UnitX()};
  double takeoff_height{1.0};
  double takeoff_duration{2.5};
  double takeoff_settle_duration{0.5};
  int revolutions{5};
  int circle_waypoints_per_revolution{16};
  double radius{1.0};
  double axial_speed{0.5};
  double entry_duration{1.5};
  double roll_duration{9.0};
  double exit_duration{1.5};
  double velocity_tracking_weight{100.0};
  double acceleration_tracking_weight{10.0};
  double jerk_tracking_weight{0.1};
  double sample_dt{0.005};
  double mass{1.5};
  double gravity{9.805};
};

// 返回与所有当前 MPC/NMPC setTrajectory() 接口直接兼容的离散参考。
OmTrajectoryResult generateFiveTurnTrajectory(
  const Eigen::Vector3d &start_position,
  const FiveTurnTrajectoryOptions &options = {});

}  // namespace px4ctrl
