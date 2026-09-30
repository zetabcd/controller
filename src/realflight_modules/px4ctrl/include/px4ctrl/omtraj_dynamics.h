#pragma once
#include <px4ctrl/omtraj.h>
namespace omtraj
{
using V9 = Eigen::Matrix<double, 9, 1>;
Eigen::Matrix3d hat(const Eigen::Vector3d & v);
Eigen::Quaterniond exp(const Eigen::Vector3d & v);
Eigen::Vector3d log(const Eigen::Quaterniond & q);
Eigen::Vector4d input(const OmTrajectoryState & x);
void setInput(OmTrajectoryState & x, const Eigen::Vector4d & u);
OmTrajectoryState retract(OmTrajectoryState x, const V9 & delta);
V9 difference(const OmTrajectoryState & x, const OmTrajectoryState & reference);
Eigen::Vector3d acceleration(const OmTrajectoryState &, const OmTrajectoryModel &);
OmTrajectoryState integrate(
  const OmTrajectoryState &, const Eigen::Vector4d & end_input,
  double duration, const OmTrajectoryModel &, int substeps);
Eigen::Matrix4d allocationInverse(const OmTrackingLimits &);
enum class ConstraintSample {All, Endpoint, Interior};
Eigen::VectorXd constraints(
  const OmTrajectoryState &, const Eigen::Vector4d & slope,
  const OmTrajectoryOptions &, double origin_z, ConstraintSample sample = ConstraintSample::All);
}
