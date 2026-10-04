#pragma once
#include <gap_planner/frame_geometry.h>
#include <gap_planner/project_flatness.h>
#include <gcopter/minco.hpp>
#include <gcopter/lbfgs.hpp>
#include <chrono>
namespace gap_planner {
// Only free transit knots, two coordinates per gate-plane knot, and positive
// segment times are optimized. MINCO eliminates ALL polynomial coefficients.
class SparseOptimizer {
  const std::vector<Gate> gates_;
  Options o_;
  FrameGeometry frames_;
  ProjectFlatness flat_;
  Eigen::Matrix4d allocation_;
  minco::MINCO_S3NU minco_;
  int pieces_,variables_;
  Eigen::VectorXd times_;
  Eigen::Matrix3Xd points_;
  std::vector<std::vector<double>> samples_;
  std::chrono::steady_clock::time_point deadline_;
  void decode(const Eigen::VectorXd &x);
  double sample(const Eigen::Vector3d &,const Eigen::Vector3d &,const Eigen::Vector3d &,
    const Eigen::Vector3d &,const Eigen::Vector3d &,int,Eigen::Vector3d &,Eigen::Vector3d &,Eigen::Vector3d &,Eigen::Vector3d &,Eigen::Vector3d &);
public:
  double weight{1e4};
  double energy_scale;
  int resolution{40},evaluations{0},iterations{0};
  Eigen::VectorXd initial;
  SparseOptimizer(const std::vector<Gate> &,const Eigen::Vector3d &,const Eigen::Vector3d &,const Options &);
  double evaluate(const Eigen::VectorXd &,Eigen::VectorXd &);
  int solve(Eigen::VectorXd &,double seconds);
  void refine(const std::vector<std::vector<double>> &samples);
  std::vector<Piece> trajectory(const Eigen::VectorXd &);
  static double callback(void *ptr,const Eigen::VectorXd &x,Eigen::VectorXd &g) {return static_cast<SparseOptimizer *>(ptr)->evaluate(x,g);}
};
}
