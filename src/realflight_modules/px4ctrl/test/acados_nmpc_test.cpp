#include <px4ctrl/acados_nmpc_solver.h>
#include <gtest/gtest.h>
#include <limits>
#include <cmath>
#include <stdexcept>

namespace
{
AcadosNmpcOptions testOptions()
{
  AcadosNmpcOptions o;
  o.solve_time_budget_ms = 1000;  // tests assert numerical behavior, not host scheduling
  return o;
}
AcadosNmpcReferences hover(const AcadosNmpcOptions & o)
{
  AcadosNmpcReferences refs(o.horizon + 1);
  for (auto & r:refs) {
    r.input.thrust_acceleration = o.gravity;
  }
  return refs;
}
}

TEST(AcadosNmpc, RuntimeHorizonGravityAndReset)
{
  for (int n:{1, 8, 12, 17}) {
    auto o = testOptions();o.horizon = n;o.prediction_dt = 0.02;o.gravity = 7.5;
    AcadosNmpcSolver solver(o);
    auto refs = hover(o);
    for (int repeat = 0; repeat < 2; ++repeat) {
      solver.reset();
      const auto u = solver.step({}, refs, 0.01);
      ASSERT_TRUE(solver.diagnostics().solved) << solver.diagnostics().status;
      EXPECT_NEAR(u.thrust_acceleration, o.gravity, 1e-5);
      EXPECT_LT(u.body_rate.norm(), 1e-6);
    }
  }
}

TEST(AcadosNmpc, ResetPreservesStepWeightsAndBounds)
{
  auto o = testOptions();o.prediction_dt = 0.017;o.horizon = 9;
  o.state_weight *= 1.7;o.terminal_weight *= 0.8;o.body_rate_max.setConstant(0.4);
  AcadosNmpcState state;state.position.x() = 0.5;
  auto refs = hover(o);
  AcadosNmpcSolver solver(o);
  const auto before = solver.step(state, refs, 0.01);
  ASSERT_TRUE(solver.diagnostics().solved);
  const double cost = solver.diagnostics().objective;
  solver.reset();
  const auto after = solver.step(state, refs, 0.01);
  ASSERT_TRUE(solver.diagnostics().solved);
  EXPECT_NEAR(after.thrust_acceleration, before.thrust_acceleration, 1e-8);
  EXPECT_LT((after.body_rate - before.body_rate).norm(), 1e-8);
  EXPECT_NEAR(solver.diagnostics().objective, cost, 1e-8);
  EXPECT_LE(after.body_rate.cwiseAbs().maxCoeff(), 0.4 + 1e-8);
}

TEST(AcadosNmpc, DiscreteCostHasNoHiddenDtScaling)
{
  for (double dt:{0.02, 0.04}) {
    auto o = testOptions();o.prediction_dt = dt;
    o.state_weight.setZero();o.state_weight[0] = 18;
    o.terminal_weight.setZero();o.terminal_weight[0] = 35;
    o.body_rate_max.setConstant(1e-9); // fixes attitude so x cannot change
    AcadosNmpcState state;state.position.x() = 1;
    AcadosNmpcSolver solver(o);
    solver.step(state, hover(o), 0.01);
    ASSERT_TRUE(solver.diagnostics().solved) << solver.diagnostics().status;
    EXPECT_NEAR(solver.diagnostics().objective, 12 * 18 + 35, 1e-4);
  }
}

TEST(AcadosNmpc, FailureDoesNotHoldOldCommandAndRecovers)
{
  auto o = testOptions();AcadosNmpcSolver solver(o);auto refs = hover(o);
  AcadosNmpcState state;state.position.x() = 0.5;
  const auto moving = solver.step(state, refs, 0.01);
  ASSERT_TRUE(solver.diagnostics().solved);
  ASSERT_GT(moving.body_rate.norm(), 0.1);
  refs.back().state.position.x() = std::numeric_limits<double>::quiet_NaN();
  for (int i = 1; i <= 5; ++i) {
    const auto fallback = solver.step({}, refs, 0.01);
    EXPECT_TRUE(solver.diagnostics().fallback);
    EXPECT_EQ(solver.diagnostics().consecutive_failures, i);
    EXPECT_LT(fallback.body_rate.norm(), 1e-8);
    EXPECT_NEAR(fallback.thrust_acceleration, o.gravity, 1e-8);
  }
  refs = hover(o);
  solver.step({}, refs, 0.01);
  EXPECT_TRUE(solver.diagnostics().solved);
  EXPECT_EQ(solver.diagnostics().consecutive_failures, 0);
}

TEST(AcadosNmpc, IterationLimitAndTimeoutUseFeedback)
{
  auto o = testOptions();o.maximum_iterations = 1;
  AcadosNmpcSolver solver(o);AcadosNmpcState s;s.position.x() = 1;
  auto u = solver.step(s, hover(o), 0.01);
  EXPECT_TRUE(solver.diagnostics().fallback);
  EXPECT_LE(solver.diagnostics().iterations, 1);
  EXPECT_TRUE(u.body_rate.allFinite());
  o.solve_time_budget_ms = 1e-9;
  AcadosNmpcSolver timed(o);
  u = timed.step(s, hover(o), 0.01);
  EXPECT_TRUE(timed.diagnostics().fallback);
  EXPECT_TRUE(u.body_rate.allFinite());
}

TEST(AcadosNmpc, InvalidInputAndConfiguration)
{
  auto o = testOptions();o.prediction_dt = 0;
  EXPECT_THROW(AcadosNmpcSolver solver(o), std::invalid_argument);
  o = testOptions();o.input_weight[0] = -1;
  EXPECT_THROW(AcadosNmpcSolver solver(o), std::invalid_argument);
  o = testOptions();AcadosNmpcSolver solver(o);
  AcadosNmpcState bad;bad.attitude.coeffs().setZero();
  auto u = solver.step(bad, hover(o), 0.01);
  EXPECT_TRUE(solver.diagnostics().fallback);
  EXPECT_TRUE(u.body_rate.allFinite());
  EXPECT_TRUE(std::isfinite(u.thrust_acceleration));
  u = solver.step({}, {}, 0.01);
  EXPECT_EQ(solver.diagnostics().status, -1);
}

TEST(AcadosNmpc, QuaternionSignAndFractionalWarmStart)
{
  auto o = testOptions();AcadosNmpcSolver solver(o);auto refs = hover(o);
  AcadosNmpcState state;state.position.x() = 0.2;
  solver.step(state, refs, 0.01);
  ASSERT_TRUE(solver.diagnostics().solved);
  state.attitude.coeffs() *= -1;
  for (auto & r:refs) {
    r.state.attitude.coeffs() *= -1;
  }
  const auto u = solver.step(state, refs, 0.01);
  EXPECT_TRUE(solver.diagnostics().solved) << solver.diagnostics().status;
  EXPECT_TRUE(u.body_rate.allFinite());
  // A gap beyond the horizon must cold-start rather than reuse stale iterates.
  solver.step(state, refs, 1.0);
  EXPECT_TRUE(solver.diagnostics().solved);
}

TEST(AcadosNmpc, RuntimeInputWeightsChangeResponse)
{
  auto o = testOptions();
  o.command_change_weight.setZero();
  AcadosNmpcState state;
  state.position.z() = -0.5;
  AcadosNmpcSolver responsive(o);
  const auto fast = responsive.step(state, hover(o), 0.01);
  ASSERT_TRUE(responsive.diagnostics().solved);
  o.input_weight *= 100;
  AcadosNmpcSolver conservative(o);
  const auto slow = conservative.step(state, hover(o), 0.01);
  ASSERT_TRUE(conservative.diagnostics().solved);
  EXPECT_GT(fast.thrust_acceleration, slow.thrust_acceleration);
  EXPECT_GT(slow.thrust_acceleration, o.gravity);
}

TEST(AcadosNmpc, ClosedLoopAt100HzWith40msPredictionGrid)
{
  auto o = testOptions();
  AcadosNmpcSolver solver(o);
  auto refs = hover(o);
  const Eigen::Vector3d target(0.5, -0.3, 0.4);
  for (auto & r : refs) {
    r.state.position = target;
  }
  AcadosNmpcState state;
  constexpr double dt = 0.01;
  int failures = 0;
  for (int k = 0; k < 400; ++k) {
    const auto u = solver.step(state, refs, dt);
    failures += solver.diagnostics().fallback ? 1 : 0;
    ASSERT_TRUE(u.body_rate.allFinite());
    ASSERT_LE(u.body_rate.cwiseAbs().maxCoeff(), o.body_rate_max.maxCoeff());
    ASSERT_GE(u.thrust_acceleration, o.thrust_acceleration_min);
    ASSERT_LE(u.thrust_acceleration, o.thrust_acceleration_max);
    // Independent ideal-rate plant: exact quaternion increment, midpoint force.
    const double speed = u.body_rate.norm();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    if (speed > 1e-12) {axis = u.body_rate / speed;}
    const auto midpoint = state.attitude *
      Eigen::Quaterniond(Eigen::AngleAxisd(0.5 * dt * speed, axis));
    const Eigen::Vector3d acceleration = midpoint * Eigen::Vector3d(0, 0, u.thrust_acceleration) -
      o.gravity * Eigen::Vector3d::UnitZ();
    state.position += dt * state.velocity + 0.5 * dt * dt * acceleration;
    state.velocity += dt * acceleration;
    state.attitude =
      (state.attitude * Eigen::Quaterniond(Eigen::AngleAxisd(dt * speed, axis))).normalized();
    state.body_rate = u.body_rate;
  }
  EXPECT_LT((state.position - target).norm(), 0.08);
  EXPECT_LT(state.velocity.norm(), 0.08);
  EXPECT_EQ(failures, 0);
}

TEST(AcadosNmpc, RateLagPlantRemainsStableAcrossPositionWeights)
{
  // Unlike the ideal-rate test, this plant has a 100/83/250 ms rate response
  // and 30 ms thrust response. Its integration is independent of generated ERK.
  for (double weight : {18.0, 20.0, 40.0}) {
    auto o = testOptions();
    o.state_weight.head<3>().setConstant(weight);
    AcadosNmpcSolver solver(o);
    auto refs = hover(o);
    const Eigen::Vector3d target(0.5, -0.3, 0.4);
    for (auto & r : refs) {
      r.state.position = target;
    }
    AcadosNmpcState state;
    double thrust = o.gravity;
    int failures = 0;
    for (int k = 0; k < 600; ++k) {
      const auto u = solver.step(state, refs, 0.01);
      failures += solver.diagnostics().fallback;
      for (int j = 0; j < 10; ++j) {
        constexpr double dt = 0.001;
        state.body_rate += dt * (u.body_rate - state.body_rate).cwiseQuotient(o.rate_time_constant);
        thrust += dt * (u.thrust_acceleration - thrust) / 0.03;
        const Eigen::Vector3d acceleration = state.attitude * Eigen::Vector3d(0, 0, thrust) -
          o.gravity * Eigen::Vector3d::UnitZ();
        state.position += dt * state.velocity + 0.5 * dt * dt * acceleration;
        state.velocity += dt * acceleration;
        const double speed = state.body_rate.norm();
        if (speed > 1e-12) {
          state.attitude = (state.attitude * Eigen::Quaterniond(
              Eigen::AngleAxisd(dt * speed, state.body_rate / speed))).normalized();
        }
      }
      ASSERT_LT(state.position.norm(), 2.0) << "Qp=" << weight;
      ASSERT_LT(state.body_rate.norm(), 6.0) << "Qp=" << weight;
    }
    EXPECT_LT((state.position - target).norm(), 0.02) << "Qp=" << weight;
    EXPECT_LT(state.velocity.norm(), 0.02) << "Qp=" << weight;
    EXPECT_EQ(failures, 0) << "Qp=" << weight;
  }
}

TEST(AcadosNmpc, MeasuredRateChangesBrakingCommand)
{
  auto o = testOptions();
  auto refs = hover(o);
  AcadosNmpcSolver quiet(o), rotating(o);
  AcadosNmpcState state;
  const auto u0 = quiet.step(state, refs, 0.01);
  state.body_rate.x() = 1.0;
  const auto u1 = rotating.step(state, refs, 0.01);
  ASSERT_TRUE(rotating.diagnostics().solved);
  EXPECT_LT(u1.body_rate.x(), u0.body_rate.x() - 0.1);
  o.rate_time_constant.x() = 0;
  EXPECT_THROW(AcadosNmpcSolver invalid(o), std::invalid_argument);
  state.body_rate.x() = std::numeric_limits<double>::quiet_NaN();
  rotating.step(state, refs, 0.01);
  EXPECT_EQ(rotating.diagnostics().status, -1);
}

TEST(AcadosNmpc, ConstantVelocityDragBalanceSurvivesReset)
{
  auto o = testOptions();
  o.linear_drag.setConstant(0.3);
  o.command_change_weight.setZero();
  AcadosNmpcSolver solver(o);
  AcadosNmpcState state;
  state.velocity.x() = 2.0;
  state.attitude = Eigen::Quaterniond(
    Eigen::AngleAxisd(
      std::atan2(0.6, o.gravity), Eigen::Vector3d::UnitY()));
  const double thrust = std::hypot(0.6, o.gravity);
  auto refs = hover(o);
  for (int k = 0; k <= o.horizon; ++k) {
    refs[k].state = state;
    refs[k].state.position = k * o.prediction_dt * state.velocity;
    refs[k].input.thrust_acceleration = thrust;
  }
  for (int repeat = 0; repeat < 2; ++repeat) {
    solver.reset();
    const auto u = solver.step(state, refs, 0.01);
    ASSERT_TRUE(solver.diagnostics().solved);
    EXPECT_LT(u.body_rate.norm(), 1e-4);
    EXPECT_NEAR(u.thrust_acceleration, thrust, 1e-4);
    EXPECT_LT(solver.diagnostics().objective, 1e-5);
  }
}
