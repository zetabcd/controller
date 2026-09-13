#ifndef PX4CTRL_MINIMUM_SNAP_TRAJECTORY_H_
#define PX4CTRL_MINIMUM_SNAP_TRAJECTORY_H_

#include <px4ctrl/omtraj.h>

#include <Eigen/Core>

#include <cstddef>
#include <vector>

// Minimum-snap 轨迹的时间分配和离散参数。多项式在每段局部归一化时间
// tau in [0, 1] 上求解，以减小长、短航段混合时的数值条件数。
struct MinimumSnapOptions
{
  double nominal_speed{2.0};       // 按距离分配段时长时使用 [m/s]
  double minimum_segment_time{0.2};
  double sample_dt{0.01};          // 输出给 MPC 缓存的离散采样周期 [s]
  double gravity{9.805};
  double yaw{0.0};                 // 当前版本整条轨迹保持固定偏航 [rad]
};

// 给定有序位置点，求解分段七次多项式：最小化所有段的积分 snap 平方，
// 同时满足位置插值、段间 v/a/jerk 连续以及首末端 v/a/jerk 为零。
// generate() 直接返回现有 MPC 能由 setTrajectory() 接收的结果类型。
class MinimumSnapTrajectory
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MinimumSnapTrajectory(
    std::size_t point_count,
    const std::vector<Eigen::Vector3d> &points,
    const MinimumSnapOptions &options = {});

  OmTrajectoryResult generate() const;

  std::size_t pointCount() const { return point_count_; }
  const std::vector<Eigen::Vector3d> &points() const { return points_; }
  const MinimumSnapOptions &options() const { return options_; }

private:
  std::size_t point_count_;
  std::vector<Eigen::Vector3d> points_;
  MinimumSnapOptions options_;
};

#endif  // PX4CTRL_MINIMUM_SNAP_TRAJECTORY_H_
