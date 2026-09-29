#ifndef PX4CTRL_FIGURE_EIGHT_TRAJECTORY_H_
#define PX4CTRL_FIGURE_EIGHT_TRAJECTORY_H_

#include <px4ctrl/omtraj.h>

#include <Eigen/Core>

// 平滑起飞 + 水平八字轨迹参数。
//
// 八字位于 takeoff 后的等高平面内，forward_axis 决定八字长轴方向；length 和
// width 是轨迹包围盒的全长/全宽。speed 是八字段的最大路径速度；生成器会根据
// 曲线尺度自动反求运行时间。轨迹末端回到八字中心并静止，MPC 在总时长之后会
// 持续采样该终点。
struct FigureEightTrajectoryOptions
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d forward_axis{Eigen::Vector3d::UnitX()};
  double takeoff_height{1.0};
  double takeoff_duration{2.5};
  double takeoff_settle_duration{0.5};
  double length{2.0};
  double width{1.2};
  double speed{1.0};
  int laps{1};
  double sample_dt{0.01};
  double gravity{9.805};
};

// 返回可直接传给 LegacyTrajectoryReference::setTrajectory() 的状态和输入
// 参考。每个采样点均包含 p、v、姿态、质量归一化推力和机体系角速度。
OmTrajectoryResult generateFigureEightTrajectory(
  const Eigen::Vector3d &start_position,
  const FigureEightTrajectoryOptions &options = {});

#endif  // PX4CTRL_FIGURE_EIGHT_TRAJECTORY_H_
