#include <px4ctrl/omtraj_dynamics.h>
#include <px4ctrl/sampled_trajectory.h>
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "../src/omtraj_qp.h"

TEST(OmtrajQp, PresolvePreservesConstrainedOptimum)
{
  using namespace omtraj::detail;
  Qp qp(3);
  qp.h.setOnes();
  qp.q << 0, 0, -4;
  qp.bound(0, 2, 2);
  qp.row({{0, 1}, {1, -2}}, 0, 0); // x1=1 after substituting x0.
  qp.bound(2, -1, 5);
  qp.row({{1, 1}, {2, 1}}, -inf, 4); // optimum x2=3.
  qp.row({{0, 1}, {2, 1}}, -inf, 10); // redundant over the box.
  Eigen::VectorXd x;
  std::string status;
  ASSERT_TRUE(solveQp(qp, x, status, 1e-8)) << status;
  EXPECT_LT((x - Eigen::Vector3d(2, 1, 3)).norm(), 1e-6);
}

TEST(OmtrajQp, FixedProblemAndContradictoryRows)
{
  using namespace omtraj::detail;
  Qp qp(2);
  qp.bound(0, 2, 2);
  qp.row({{0, 1}, {1, -2}}, 0, 0);
  Eigen::VectorXd x;
  std::string status;
  ASSERT_TRUE(solveQp(qp, x, status, 1e-8));
  EXPECT_LT((x - Eigen::Vector2d(2, 1)).norm(), 1e-10);
  qp.row({{0, 1}, {1, 1}}, 4, inf);
  EXPECT_FALSE(solveQp(qp, x, status, 1e-8));
}

TEST(Omtraj, HoverAndPointToPointAreValidated)
{
  OmTrajectoryOptions o;
  o.intervals = 20;
  o.integration_substeps = 1; // Independent audit must trigger refinement.
  o.max_scp_iterations = 60;
  o.initial_speed = .6;
  o.virtual_control_weight = 10;
  o.cstc_merit_weight = 10;
  OmTrajectoryBoundary a, b;
  a.position.z() = 1;
  b = a;
  b.position.x() = 1;
  auto r = OmTrajectoryOptimizer(o).optimize(a, b, {});
  ASSERT_TRUE(r.success) << r.status;
  EXPECT_GT(r.integration_substeps, 1);
  EXPECT_TRUE(validateOmTrajectory(r, o).valid);
  EXPECT_LT((r.states.back().position - b.position).norm(), 1e-5);
  auto end = OmTrajectoryOptimizer::sample(r, r.total_time + 1);
  EXPECT_TRUE(end.body_rate.isZero());
  EXPECT_NEAR(end.thrust_acceleration, o.model.gravity, 1e-8);
}
TEST(Omtraj, IntegratedReferenceHasConsistentDerivatives)
{
  OmTrajectoryResult r;
  r.total_time = .1;
  r.integration_substeps = 10;
  OmTrajectoryState x;
  x.body_rate = {.3, -.2, .1};
  x.thrust_acceleration = 11;
  Eigen::Vector4d u;
  u << 12, .5, .1, -.1;
  auto y = omtraj::integrate(x, u, .1, r.model, 10);
  r.states = {x, y};
  double t = .05, h = 1e-5;
  auto a = OmTrajectoryOptimizer::sample(r, t - h), b = OmTrajectoryOptimizer::sample(r, t + h),
    s = OmTrajectoryOptimizer::sample(r, t);
  EXPECT_LT(((b.position - a.position) / (2 * h) - s.velocity).norm(), 1e-6);
  EXPECT_LT(((b.velocity - a.velocity) / (2 * h) - omtraj::acceleration(s, r.model)).norm(), 1e-6);
  EXPECT_LT(
    (omtraj::log(a.attitude.conjugate() * b.attitude) / (2 * h) - s.body_rate).norm(),
    1e-6);
}
TEST(Omtraj, InvalidConfigurationRejected)
{
  OmTrajectoryOptions o;
  o.minimum_time = -1;
  EXPECT_THROW(
    OmTrajectoryOptimizer{o},
    std::invalid_argument);
  o = {};
  o.tracking.motor_max = -1;
  EXPECT_THROW(OmTrajectoryOptimizer{o}, std::invalid_argument);
}

TEST(Omtraj, OrderedWaypointsAndModelAwareCsv)
{
  OmTrajectoryOptions o;
  o.intervals = 24;
  o.max_scp_iterations = 90;
  o.initial_speed = .7;
  o.model.linear_drag = {.26 / .811, .28 / .811, .42 / .811};
  o.model.horizontal_lift = .01 / .811;
  OmTrajectoryBoundary a, b;
  a.position.z() = 1;
  b = a;
  b.position = {1.5, .3, 1.1};
  OmWaypoints wp(2);
  wp[0].position = {.4, .3, 1.1};
  wp[1].position = {1, -.1, 1.2};
  auto result = OmTrajectoryOptimizer(o).optimize(a, b, wp);
  ASSERT_TRUE(result.success) << result.status;
  auto audit = validateOmTrajectory(result, o, wp);
  ASSERT_TRUE(audit.valid) << audit.reason;
  ASSERT_EQ(audit.waypoint_times.size(), 2u);
  EXPECT_LT(
    audit.waypoint_times[0],
    audit.waypoint_times[1]);
  std::swap(wp[0], wp[1]);
  EXPECT_FALSE(validateOmTrajectory(result, o, wp).valid);
  const auto file = std::filesystem::path(testing::TempDir()) / "omtraj_v2_test.csv";
  std::string error;
  ASSERT_TRUE(saveOmTrajectoryCsv(result, file.string(), &error)) << error;
  auto loaded = loadOmTrajectoryCsv(file.string());
  ASSERT_TRUE(loaded.success) << loaded.status;
  EXPECT_TRUE(loaded.dynamics_validated);
  EXPECT_FALSE(loaded.converged);
  auto source = px4ctrl::loadOmTrajectoryReference(file.string(), o.model.gravity);
  const double t = .47 * result.total_time, h = 1e-5;
  auto x = source->evaluate(t), before = source->evaluate(t - h), after = source->evaluate(t + h);
  EXPECT_TRUE(x.aerodynamics_included);
  EXPECT_TRUE(x.angular_acceleration_valid);
  EXPECT_TRUE(x.thrust_rate_valid);
  EXPECT_FALSE(x.kinematics_valid);
  EXPECT_LT(((after.position - before.position) / (2 * h) - x.velocity).norm(), 1e-5);
  EXPECT_LT(((after.velocity - before.velocity) / (2 * h) - x.acceleration).norm(), 1e-5);
  EXPECT_LT(((after.body_rate - before.body_rate) / (2 * h) - x.body_acceleration).norm(), 1e-5);
  const auto same = px4ctrl::compensateReferenceAerodynamics(
    x, o.model.gravity,
    o.model.linear_drag,
    o.model.horizontal_lift);
  EXPECT_LT(same.attitude.angularDistance(x.attitude), 1e-12);
  EXPECT_THROW(
    px4ctrl::compensateReferenceAerodynamics(
      x, o.model.gravity, Eigen::Vector3d::Zero(),
      0), std::invalid_argument);
  loaded.states[3].position.x() += .05;
  EXPECT_FALSE(validateOmTrajectory(loaded, o).valid);
  EXPECT_FALSE(saveOmTrajectoryCsv(loaded, file.string(), &error));
  std::filesystem::remove(file);
}
TEST(Omtraj, LowSpeedEnvelopeChangesFeasibleFlightTime)
{
  OmTrajectoryOptions o;
  o.intervals = 16;
  o.initial_speed = .5;
  o.max_scp_iterations = 80;
  o.tracking.speed_max = .6;
  OmTrajectoryBoundary a, b;
  a.position.z() = 1;
  b = a;
  b.position.x() = 1;
  auto r = OmTrajectoryOptimizer(o).optimize(a, b, {});
  ASSERT_TRUE(r.success) << r.status;
  EXPECT_GE(r.total_time, 1 / .6 - 1e-3);
  for (double t = 0; t < r.total_time; t += .003) {
    EXPECT_LE(OmTrajectoryOptimizer::sample(r, t).velocity.norm(), .6001);
  }
  o.tracking.speed_max = .1;
  EXPECT_FALSE(validateOmTrajectory(r, o).valid);
}
TEST(Omtraj, BodyZConeAndMotorTorqueAreIndependentHardLimits)
{
  OmTrajectoryOptions o;
  o.tracking.maximum_tilt = 1.2;
  OmTrajectoryState x;
  x.attitude = Eigen::Quaterniond(Eigen::AngleAxisd(1.4, Eigen::Vector3d::UnitX()));
  EXPECT_GT(omtraj::constraints(x, Eigen::Vector4d::Zero(), o, 0).maxCoeff(), 0);
  x.attitude.setIdentity();
  o.tracking.angular_acceleration_max.setConstant(1000);
  o.tracking.motor_max = 3;
  Eigen::Vector4d slope;
  slope << 0, 0, 0, 100;
  EXPECT_GT(omtraj::constraints(x, slope, o, 0).maxCoeff(), 0);
  o.tracking.motor_constraints = false;
  EXPECT_LE(omtraj::constraints(x, slope, o, 0).maxCoeff(), 0);
}

TEST(Omtraj, LegacyCsvIsInspectionOnly)
{
  const auto file = std::filesystem::path(testing::TempDir()) / "omtraj_legacy_test.csv";
  std::ofstream out(file);
  out << "0,0,0,0,0,0,0,1,0,0,0,9.805,0,0,0\n"
      << "0.1,0,0,0,0,0,0,1,0,0,0,9.805,0,0,0\n";
  out.close();
  const auto r = loadOmTrajectoryCsv(file.string());
  ASSERT_TRUE(r.success);
  EXPECT_FALSE(r.dynamics_validated);
  EXPECT_FALSE(r.converged);
  EXPECT_THROW(px4ctrl::loadOmTrajectoryReference(file.string(), 9.805), std::invalid_argument);
  std::filesystem::remove(file);
}
