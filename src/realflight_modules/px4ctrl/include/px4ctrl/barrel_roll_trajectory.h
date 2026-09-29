#ifndef PX4CTRL_BARREL_ROLL_TRAJECTORY_H_
#define PX4CTRL_BARREL_ROLL_TRAJECTORY_H_

#include <px4ctrl/omtraj.h>

#include <Eigen/Core>

// Barrel-roll 轨迹参数。轨迹由垂直起飞、稳定悬停、平滑进入、完整滚转和
// 平滑退出五个阶段组成；roll_axis 在 ENU 世界系中表达且必须位于水平面。
struct BarrelRollTrajectoryOptions
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d roll_axis{Eigen::Vector3d::UnitX()};
  double takeoff_height{1.0};
  double takeoff_duration{2.5};
  double takeoff_settle_duration{0.5};
  double radius{1.0};
  double axial_speed{0.5};
  double entry_duration{1.5};
  double roll_duration{3.5};
  double exit_duration{1.5};
  int turns{1};
  int polynomial_segments_per_turn{32};
  double sample_dt{0.01};
  double gravity{9.805};
};

// 生成与 LegacyTrajectoryReference::setTrajectory() 直接兼容的完整离散轨迹。
// 每个状态同时包含 p、v、R、质量归一化推力和机体系角速度，因此该轨迹
// 不只是用于显示的位置曲线，而是可以直接作为 MPC 的状态/输入参考。
OmTrajectoryResult generateBarrelRollTrajectory(
  const Eigen::Vector3d &start_position,
  const BarrelRollTrajectoryOptions &options = {});

#endif  // PX4CTRL_BARREL_ROLL_TRAJECTORY_H_
