#include <px4ctrl/ommpc_solver.h>
#include <px4ctrl/trajectory.h>
#include <gtest/gtest.h>
#include <limits>

namespace
{
px4ctrl::ReferenceWindow hover(const OmMpcOptions & o)
{
  px4ctrl::ReferencePoint r;r.position.z() = 1;
  return px4ctrl::extrapolateFlatReference(r, 0, o.horizon, o.prediction_dt, o.gravity);
}
OmMpcOptions testOptions()
{
  OmMpcOptions o;o.solve_time_budget_ms = 1000;return o;
}
}
TEST(OmMpc, RejectsInvalidConfiguration)
{
  auto o = testOptions();o.input_weight(0) = -1;
  EXPECT_THROW(OmMpcSolver solver(o), std::invalid_argument);
  o = testOptions();o.rate_time_constant.x() = 0;
  EXPECT_THROW(OmMpcSolver solver(o), std::invalid_argument);
  o = testOptions();o.prediction_dt = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(OmMpcSolver solver(o), std::invalid_argument);
}
TEST(OmMpc, HoverAndQuaternionSign)
{
  const auto o = testOptions();OmMpcSolver a(o), b(o);
  OmMpcState s;s.position.z() = 1;
  const auto u = a.step(s, hover(o), .01);s.attitude.coeffs() *= -1;
  const auto v = b.step(s, hover(o), .01);
  ASSERT_TRUE(a.diagnostics().solved);ASSERT_TRUE(b.diagnostics().solved);
  EXPECT_NEAR(u.input(0), o.gravity, 1e-5);
  EXPECT_LT(u.input.tail<3>().norm(), 1e-5);
  EXPECT_LT((u.input - v.input).norm(), 1e-8);
  const auto repeated = a.step(s, hover(o), 0.0);
  EXPECT_TRUE(a.diagnostics().solved);
  EXPECT_LT((u.input - repeated.input).norm(), 1e-6);
}
TEST(OmMpc, PredictsActuatorLagAndInvertedThrust)
{
  const auto o = testOptions();OmMpcState s;
  const Eigen::Vector4d u(15, 2, 0, 0);
  const auto n = ommpcPredict(s, u, Eigen::Vector3d::Zero(), .01, o);
  EXPECT_GT(n.body_rate.x(), 0);EXPECT_LT(n.body_rate.x(), u(1));
  EXPECT_GT(n.thrust_acceleration, o.gravity);EXPECT_LT(n.thrust_acceleration, u(0));
  s.attitude = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  const auto inverted = ommpcPredict(s, {o.gravity, 0, 0, 0}, Eigen::Vector3d::Zero(), .01, o);
  EXPECT_NEAR(inverted.velocity.z(), -2 * o.gravity * .01, 1e-8);
  EXPECT_NEAR(inverted.attitude.norm(), 1, 1e-12);
}
TEST(OmMpc, TracksPositionWithFiniteRateResponse)
{
  auto o = testOptions();o.linear_drag = {.32, .35, .52};
  OmMpcSolver solver(o);OmMpcState s;s.position = {.3, -.2, 1.2};
  auto w = hover(o);
  for (int i = 0; i < 400; ++i) {
    w.stamp = i * .01;
    const auto u = solver.step(s, w, .01);
    ASSERT_TRUE(solver.diagnostics().solved) << solver.diagnostics().status;
    s = ommpcPredict(s, u.input, s.attitude * u.angular_acceleration, .01, o);
    ASSERT_TRUE(s.position.allFinite());
  }
  EXPECT_LT((s.position - Eigen::Vector3d(0, 0, 1)).norm(), .02);
  EXPECT_LT(s.velocity.norm(), .03);
}
TEST(OmMpc, BoundsAndFailureRecovery)
{
  auto o = testOptions();o.body_rate_max.setConstant(.5);
  OmMpcSolver solver(o);OmMpcState s;s.position = {2, -2, 1};
  const auto w = hover(o);auto u = solver.step(s, w, .01);
  ASSERT_TRUE(solver.diagnostics().solved);
  EXPECT_LE(u.input.tail<3>().cwiseAbs().maxCoeff(), .5);
  EXPECT_GE(u.input(0), o.thrust_acceleration_min);
  EXPECT_LE(u.input(0), o.thrust_acceleration_max);
  s.position = {-2, 2, 1};u = solver.step(s, w, .3);
  EXPECT_TRUE(solver.diagnostics().fallback);EXPECT_TRUE(u.input.allFinite());
  EXPECT_EQ(solver.diagnostics().consecutive_failures, 1);
  EXPECT_TRUE(u.angular_acceleration.isZero());
  solver.reset();u = solver.step(s, w, .01);
  EXPECT_TRUE(solver.diagnostics().solved);
  EXPECT_EQ(solver.diagnostics().consecutive_failures, 0);
}
TEST(OmMpc, FullFlipReferenceHasNoEulerSingularity)
{
  const auto o = testOptions();OmMpcSolver solver(o);
  px4ctrl::AnalyticTrajectoryOptions config;config.path = px4ctrl::AnalyticPath::VerticalCircle;
  auto source = std::make_shared<px4ctrl::AnalyticTrajectory>(config);
  px4ctrl::TrajectoryPlayer player;player.start(source, 0, Eigen::Vector3d::Zero(), 0);
  for (double t = source->mainStart(); t < source->mainEnd(); t += .01) {
    const auto w = player.sample(t, o.horizon, o.prediction_dt);const auto & r = w.points.front();
    const auto u = solver.step(
      {r.position, r.velocity, r.attitude, r.body_rate,
        r.thrust_acceleration}, w, .01);
    ASSERT_TRUE(solver.diagnostics().solved) << t << " status " << solver.diagnostics().status;
    EXPECT_TRUE(u.input.allFinite());EXPECT_TRUE(u.angular_acceleration.allFinite());
  }
}

TEST(OmMpc, InvalidStateBudgetAndReferenceAreExplicit)
{
  auto o = testOptions();OmMpcSolver solver(o);OmMpcState s;s.position.z() = 1;
  auto w = hover(o);s.body_rate.x() = std::numeric_limits<double>::quiet_NaN();
  auto u = solver.step(s, w, .01);
  EXPECT_TRUE(solver.diagnostics().fallback);EXPECT_FALSE(solver.diagnostics().solved);
  EXPECT_TRUE(u.input.allFinite());EXPECT_TRUE(u.input.tail<3>().isZero());
  s.body_rate.setZero();w.points.pop_back();
  EXPECT_THROW(solver.step(s, w, .01), std::invalid_argument);
  o.solve_time_budget_ms = 1e-6;OmMpcSolver tiny(o);
  u = tiny.step(s, hover(o), .01);
  EXPECT_EQ(tiny.diagnostics().status, -3);EXPECT_TRUE(tiny.diagnostics().fallback);
  EXPECT_TRUE(u.input.allFinite());
}

TEST(OmMpc, AngularFeedforwardDoesNotDoubleCompensateRateLag)
{
  const auto o = testOptions();OmMpcSolver solver(o);
  px4ctrl::ReferencePoint r;r.position.z() = 1;r.yaw_acceleration = .7;
  const auto w = px4ctrl::extrapolateFlatReference(r, 0, o.horizon, o.prediction_dt, o.gravity);
  OmMpcState s;s.position = r.position;
  const auto u = solver.step(s, w, .01);
  ASSERT_TRUE(solver.diagnostics().solved);
  EXPECT_NEAR(u.angular_acceleration.z(), .7, 1e-10);
  // A second inverse-lag term would add tau_z * .7 = .175 rad/s.
  EXPECT_LT(std::abs(u.input(3)), .05);
}
