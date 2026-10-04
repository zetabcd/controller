#pragma once
#include <gap_planner/core.h>
namespace gap_planner {
struct FrameBox {Eigen::Vector3d center,half;Eigen::Matrix3d rotation;std::string id;};
struct Separation {
  double distance{-std::numeric_limits<double>::infinity()};
  Eigen::Vector3d position_gradient{Eigen::Vector3d::Zero()};
  Eigen::Vector4d quaternion_gradient{Eigen::Vector4d::Zero()};
};
// A separating-axis distance is a conservative clearance certificate. For a
// polytope, the complete face/edge axis set also detects every intersection.
class FrameGeometry {
  BodyVertices vertices_,normals_,edges_;
  Eigen::Vector3d axes_;
public:
  std::vector<FrameBox> boxes;
  double radius{0};
  FrameGeometry(const std::vector<Gate> &gates,const Options &options);
  Separation separation(const FrameBox &,const Eigen::Vector3d &,const Eigen::Vector4d &,bool gradient=true) const;
};
void validateTask(const std::vector<Gate> &,const Eigen::Vector3d &,const Eigen::Vector3d &,const Options &);
}
