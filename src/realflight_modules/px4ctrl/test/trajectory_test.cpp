#include <px4ctrl/trajectory.h>
#include <px4ctrl/sampled_trajectory.h>
#include <gtest/gtest.h>
#include <px4ctrl/omtraj.h>
#include <filesystem>
#include <array>
#include <cmath>
#include <limits>

using namespace px4ctrl;
namespace
{
constexpr double pi = 3.14159265358979323846;
const std::array<AnalyticPath, 4> paths{AnalyticPath::HorizontalCircle,
  AnalyticPath::VerticalCircle, AnalyticPath::Helix, AnalyticPath::FigureEight};
std::array<Eigen::Vector3d, 5> derivatives(const ReferencePoint & r)
{return {r.position, r.velocity, r.acceleration, r.jerk, r.snap};}
}

TEST(Trajectory, AllProfilesHaveConsistentAnalyticDerivatives)
{
  for (auto path:paths) {
    AnalyticTrajectoryOptions o;o.path = path;AnalyticTrajectory trajectory(o);
    for (double t = 0.017; t < trajectory.duration() - 0.02; t += 0.073) {
      constexpr double h = 1e-5;
      const auto a = trajectory.evaluate(t - h), r = trajectory.evaluate(t),
        b = trajectory.evaluate(t + h);
      ASSERT_TRUE(r.kinematics_valid);ASSERT_TRUE(r.angular_acceleration_valid);
      const auto da = derivatives(a), d = derivatives(r), db = derivatives(b);
      for (int k = 0; k < 4; ++k) {
        EXPECT_LT(
          ((db[k] - da[k]) / (2 * h) - d[k + 1]).norm(),
          2e-3) << int(path) << " t=" << t << " derivative=" << k;
      }
      const Eigen::Matrix3d W = r.attitude.toRotationMatrix().transpose() *
        (b.attitude.toRotationMatrix() - a.attitude.toRotationMatrix()) / (2 * h);
      EXPECT_LT((r.body_rate - Eigen::Vector3d(W(2, 1), W(0, 2), W(1, 0))).norm(), 2e-5);
      EXPECT_LT((r.body_acceleration - (b.body_rate - a.body_rate) / (2 * h)).norm(), 2e-3);
      EXPECT_LT(
        (r.acceleration - (r.attitude * Eigen::Vector3d(0, 0, r.thrust_acceleration) -
        o.gravity * Eigen::Vector3d::UnitZ())).norm(), 1e-10);
    }
  }
}

TEST(Trajectory, C4ConnectionsAndExplicitTerminalHold)
{
  for (auto path:paths) {
    AnalyticTrajectoryOptions o;o.path = path;AnalyticTrajectory trajectory(o);
    for (double t:trajectory.boundaries()) {
      const auto a = trajectory.evaluate(t - 1e-8), b = trajectory.evaluate(t + 1e-8);
      const auto da = derivatives(a), db = derivatives(b);
      for (int k = 0; k <= 4; ++k) {
        EXPECT_LT((da[k] - db[k]).norm(), 0.005) << int(path) << " t=" << t << " derivative=" << k;
      }
      EXPECT_LT(a.attitude.angularDistance(b.attitude), 1e-5);
      EXPECT_LT((a.body_rate - b.body_rate).norm(), 1e-4);
      EXPECT_LT((a.body_acceleration - b.body_acceleration).norm(), 0.005);
    }
    const auto end = trajectory.evaluate(trajectory.duration() + 10);
    EXPECT_TRUE(end.velocity.isZero());EXPECT_TRUE(end.acceleration.isZero());
    EXPECT_TRUE(end.jerk.isZero());EXPECT_TRUE(end.snap.isZero());
    EXPECT_TRUE(end.body_rate.isZero());EXPECT_TRUE(end.body_acceleration.isZero());
    EXPECT_NEAR(end.thrust_acceleration, o.gravity, 1e-12);
  }
}

TEST(Trajectory, InversionAndHelixPitchMatchClosedForm)
{
  for (auto path:{AnalyticPath::VerticalCircle, AnalyticPath::Helix}) {
    AnalyticTrajectoryOptions o;o.path = path;AnalyticTrajectory trajectory(o);
    const double w = std::sqrt(o.centripetal_g * o.gravity / o.radius),
      A = o.centripetal_g * o.gravity;
    const auto start = trajectory.evaluate(trajectory.mainStart());
    const auto lap = trajectory.evaluate(trajectory.mainStart() + 2 * pi / w);
    EXPECT_NEAR(
      (lap.position - start.position).x(), path == AnalyticPath::Helix ? 0.5 : 0.0,
      1e-10);
    for (double theta = 0; theta < 2 * pi * o.turns; theta += 0.03) {
      const auto r = trajectory.evaluate(trajectory.mainStart() + theta / w);
      const double D = A * A + o.gravity * o.gravity + 2 * A * o.gravity * std::cos(theta);
      const double rate = w * A * (A + o.gravity * std::cos(theta)) / D;
      EXPECT_NEAR(r.thrust_acceleration, std::sqrt(D), 1e-10);
      EXPECT_NEAR(path == AnalyticPath::Helix ? r.body_rate.x() : -r.body_rate.y(), rate, 1e-10);
    }
    const auto top = trajectory.evaluate(trajectory.mainStart() + pi / w);
    EXPECT_LT((top.attitude * Eigen::Vector3d::UnitZ()).z(), -0.999999);
    const double bottom = start.position.z();
    EXPECT_NEAR(top.position.z() - bottom, 2.0, 1e-12);
  }
}

TEST(Trajectory, NominalDefaultsPassSampledActuatorAudit)
{
  for (auto path:paths) {
    AnalyticTrajectoryOptions o;o.path = path;AnalyticTrajectory trajectory(o);
    TrajectoryLimits limits;
    // Runtime YAML motor model, including idle and 70% single-motor budget.
    const double ct = .1862 * 4 * 1.1 * std::pow(.06475, 4) / (pi * pi);
    limits.motor_min = ct * 291.082 * 291.082;
    limits.motor_max = .7 * ct * std::pow(-92.209 + 3995.891 + 291.082, 2);
    limits.torque_to_thrust = 2 * .06475 * .0164 / .1862;
    const auto audit = auditTrajectory(trajectory, limits);
    EXPECT_TRUE(audit.valid) << int(path) << ": " << audit.reason << " at " <<
      audit.first_failure_time;
    limits.rate_max.setConstant(.01);
    EXPECT_FALSE(auditTrajectory(trajectory, limits, .005).valid);
  }
}

TEST(Trajectory, PlayerUsesExactPredictionTimesAndRigidFrameTransform)
{
  auto source = std::make_shared<AnalyticTrajectory>(AnalyticTrajectoryOptions{});
  TrajectoryPlayer player;
  const Eigen::Vector3d offset(1, 2, 3);const double yaw = .7;
  player.start(source, 100, offset, yaw);
  const auto window = player.sample(104, 12, .04);
  ASSERT_EQ(window.points.size(), 13u);
  const Eigen::Quaterniond R(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  for (int k = 0; k <= 12; ++k) {
    const auto r = source->evaluate(4 + k * .04), s = window.points[k];
    EXPECT_LT((s.position - offset - R * r.position).norm(), 1e-10);
    EXPECT_LT((s.snap - R * r.snap).norm(), 1e-10);
    EXPECT_LT((s.body_acceleration - r.body_acceleration).norm(), 1e-10);
  }
  EXPECT_THROW(player.sample(103, 12, .04), std::invalid_argument);
  EXPECT_THROW(player.sample(99, 12, .04), std::invalid_argument);
  player.clear();EXPECT_FALSE(player.active());
  EXPECT_THROW(player.sample(104, 12, .04), std::invalid_argument);
}

TEST(Trajectory, InvalidParametersAndClockAreRejected)
{
  AnalyticTrajectoryOptions o;o.radius = 1.01;
  EXPECT_THROW(AnalyticTrajectory{o}, std::invalid_argument);
  o = {};o.path = AnalyticPath::Helix;o.centripetal_g = 1;
  EXPECT_THROW(AnalyticTrajectory{o}, std::invalid_argument);
  o = {};o.speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(AnalyticTrajectory{o}, std::invalid_argument);
  o = {};AnalyticTrajectory t(o);
  EXPECT_THROW(t.evaluate(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(Trajectory, SampledSourceHasExplicitHoldAndNoInventedSnap)
{
  ReferencePoint r;r.full_state = true;r.thrust_acceleration = 9.805;r.body_rate.x() = .1;
  TimedReferences points{{0, r}, {1, r}};points.back().point.position.x() = 1;
  SampledTrajectory t(points, 9.805);
  EXPECT_FALSE(t.evaluate(.5).kinematics_valid);
  EXPECT_NEAR(t.evaluate(.5).position.x(), .5, 1e-12);
  EXPECT_TRUE(t.evaluate(2).body_rate.isZero());
  EXPECT_TRUE(t.evaluate(2).body_acceleration.isZero());
  points.back().time = 0;EXPECT_THROW((SampledTrajectory{points, 9.805}), std::invalid_argument);
}

TEST(Trajectory, AerodynamicCorrectionPreservesInversionAndAnalyticFeedforward)
{
  AnalyticTrajectoryOptions o;o.path = AnalyticPath::VerticalCircle;AnalyticTrajectory t(o);
  const Eigen::Vector3d drag(.15, .15, .1);
  const auto sample = [&](double time) {
      return compensateReferenceAerodynamics(t.evaluate(time), o.gravity, drag, .01);
    };
  for (double time = t.mainStart() + .01; time < t.mainEnd() - .01; time += .017) {
    const auto r = sample(time), a = sample(time - 1e-5), b = sample(time + 1e-5);
    EXPECT_LT(r.attitude.angularDistance(t.evaluate(time).attitude), .15);
    const Eigen::Matrix3d W = r.attitude.toRotationMatrix().transpose() *
      (b.attitude.toRotationMatrix() - a.attitude.toRotationMatrix()) / 2e-5;
    EXPECT_LT((r.body_rate - Eigen::Vector3d(W(2, 1), W(0, 2), W(1, 0))).norm(), 2e-5);
    EXPECT_LT((r.body_acceleration - (b.body_rate - a.body_rate) / 2e-5).norm(), 2e-3);
  }
}

TEST(Trajectory, QuaternionLiftStaysContinuousAcrossRevolutionsAndHold)
{
  AnalyticTrajectoryOptions o;o.path = AnalyticPath::Helix;o.turns = 3;
  AnalyticTrajectory trajectory(o);
  auto previous = trajectory.evaluate(0).attitude;
  for (double t = .001; t < trajectory.duration() + 1; t += .001) {
    const auto q = trajectory.evaluate(t).attitude;
    EXPECT_GT(previous.dot(q), .99);previous = q;
  }
}

TEST(Trajectory, OmtrajCsvPersistenceConvertsAtSourceBoundary)
{
  OmTrajectoryResult original;original.success = true;
  OmTrajectoryState first;first.position = {3, 4, 1};
  first.attitude = Eigen::Quaterniond(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()));
  auto last = first;last.time = 1;last.position.x() += 1;
  original.states = {first, last};original.total_time = 1;
  const auto file = std::filesystem::path(testing::TempDir()) / "px4ctrl_trajectory_roundtrip.csv";
  std::string error;
  ASSERT_TRUE(saveOmTrajectoryCsv(original, file.string(), &error)) << error;
  const auto trajectory = loadOmTrajectoryReference(file.string(), 9.805);
  EXPECT_LT(trajectory->evaluate(0).position.norm(), 1e-12);
  EXPECT_LT(trajectory->evaluate(0).attitude.angularDistance(Eigen::Quaterniond::Identity()),
    1e-12);
  const auto endpoint = trajectory->evaluate(2);
  EXPECT_LT(
    (endpoint.position - first.attitude.conjugate() * Eigen::Vector3d::UnitX()).norm(), 1e-12);
  EXPECT_TRUE(endpoint.body_rate.isZero());
  std::filesystem::remove(file);
}
