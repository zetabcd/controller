#pragma once
#include <px4ctrl/external_trajectory.h>
#include <gap_planner/body_geometry.h>
#include <Eigen/Geometry>
#include <string>
#include <stdexcept>

namespace gap_planner
{
struct Gate
{
  std::string id;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  // Local X is through direction; Y is opening width; Z is opening height.
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  double width{0.8}, height{0.3}, thickness{0.04}, frame_width{0.05};
};
struct Options
{
  Eigen::Vector3d body{0.263539,0.263539,0.070711};
  BodyVertices vertices; // empty: ellipsoid; otherwise convex hull of body-frame points
  double margin{0.02};
  double optimization_buffer{0.002}; // additional numerical reserve, not physical margin
  double speed{2.0}, rate{6.0}, thrust_min{3.0}, thrust_max{18.0};
  double tilt{1.52}, time_weight{100.0}, audit_dt{0.001};
  double solve_budget{5.0};
  px4ctrl::ExternalModel model;
  px4ctrl::TrajectoryLimits execution_limits;
};
struct Piece
{
  double duration;
  Eigen::Matrix<double,3,6> coefficients; // ascending local-time powers
};
struct PlanTiming
{
  double setup_ms{0}, optimize_ms{0}, audit_ms{0}, total_ms{0};
  std::vector<double> stage_ms;
  std::vector<int> stage_status;
  std::vector<std::string> audit_details;
  int variables{0},pieces{0},evaluations{0},iterations{0};
  std::string phase{"setup"};
};
std::string timingText(const PlanTiming &timing);
class PlanError : public std::runtime_error
{
public:
  PlanTiming timing;
  PlanError(const std::string &reason,const PlanTiming &value)
    : std::runtime_error(reason),timing(value) {}
};
struct GateCrossing
{
  std::string id;
  double time{0}, roll_deg{0}, tilt_deg{0};
};
struct Result
{
  std::vector<Piece> pieces;
  double min_clearance{0}, duration{0}, max_speed{0}, max_rate{0};
  double path_length{0},endpoint_distance{0};
  double backward_distance{0};
  PlanTiming timing;
  std::vector<GateCrossing> crossings;
};
px4ctrl::ReferencePoint flatPoint(const Piece &piece,double time);
Result plan(const std::vector<Gate> &gates,std::size_t count,const Eigen::Vector3d &start,
  const Eigen::Vector3d &goal,const Options &options);
}
