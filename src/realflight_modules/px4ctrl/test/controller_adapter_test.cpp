#include <px4ctrl/acados_nmpc.h>
#include <px4ctrl/trajectory.h>
#include <gtest/gtest.h>
#include <stdexcept>

TEST(ControllerAdapter, FeedforwardReachesOutputAndClearsOnManualAndFailure)
{
  AcadosNmpcOptions options;options.solve_time_budget_ms = 1000;
  AcadosNmpcControl controller;controller.configure(options, 1.5);
  px4ctrl::ReferencePoint flat;flat.yaw_acceleration = 0.7;
  const auto window = px4ctrl::extrapolateFlatReference(
    flat, 12, options.horizon, options.prediction_dt, options.gravity);
  px4ctrl::ControlModeReference mode;mode.fsm_state = 3;
  AcadosNmpcState state;
  Control_Setpoint_t out;
  controller.calculate(window, mode, state, 12, 0.01, out);
  ASSERT_TRUE(controller.diagnostics().solved) << controller.diagnostics().status;
  EXPECT_TRUE(out.rate_dot_ref_valid);
  EXPECT_NEAR(out.rate_dot_ref.z(), 0.7, 1e-12);
  EXPECT_THROW(controller.calculate(window, mode, state, 13, 0.01, out), std::invalid_argument);
  mode.fsm_state = 1;mode.throttle = 0.5;
  controller.calculate({}, mode, state, 12, 0.01, out);
  EXPECT_FALSE(out.rate_dot_ref_valid);
  EXPECT_TRUE(out.rate_dot_ref.isZero());
  options.solve_time_budget_ms = 1e-9;controller.configure(options, 1.5);mode.fsm_state = 3;
  controller.calculate(window, mode, state, 12, 0.01, out);
  ASSERT_TRUE(controller.diagnostics().fallback);
  EXPECT_FALSE(out.rate_dot_ref_valid);
  EXPECT_TRUE(out.rate_dot_ref.isZero());
}

TEST(ControllerAdapter, AnalyticInvertedReferenceReachesInnerLoopWithDrag)
{
  for (auto path : {px4ctrl::AnalyticPath::VerticalCircle, px4ctrl::AnalyticPath::Helix}) {
    px4ctrl::AnalyticTrajectoryOptions trajectory_options;trajectory_options.path = path;
    auto trajectory = std::make_shared<px4ctrl::AnalyticTrajectory>(trajectory_options);
    const double phase_rate = std::sqrt(trajectory_options.centripetal_g * trajectory_options.gravity);
    const double now = trajectory->mainStart() + 2.8 / phase_rate;
    px4ctrl::TrajectoryPlayer player;player.start(trajectory, 0, Eigen::Vector3d::Zero(), 0.7);
    AcadosNmpcOptions options;options.solve_time_budget_ms = 1000;
    options.linear_drag = {0.15, 0.15, 0.1};options.horizontal_lift = 0.01;
    const auto window = player.sample(now, options.horizon, options.prediction_dt);
    const auto & first = window.points.front();
    ASSERT_LT((first.attitude * Eigen::Vector3d::UnitZ()).z(), 0);
    AcadosNmpcState state{first.position, first.velocity, first.attitude, first.body_rate};
    px4ctrl::ControlModeReference mode;mode.fsm_state = 3;
    AcadosNmpcControl controller;controller.configure(options, 0.811);
    Control_Setpoint_t out;
    controller.calculate(window, mode, state, now, 0.01, out);
    ASSERT_TRUE(controller.diagnostics().solved) << controller.diagnostics().status;
    ASSERT_TRUE(out.rate_dot_ref_valid);
    const auto corrected = px4ctrl::compensateReferenceAerodynamics(
      first, options.gravity, options.linear_drag, options.horizontal_lift);
    EXPECT_LT((out.rate_dot_ref - px4ctrl::angularFeedforward(corrected, state.attitude)).norm(), 1e-10);
    EXPECT_LT(out.q.angularDistance(corrected.attitude), 1e-10);
    EXPECT_TRUE(out.bodyrates.allFinite());
    EXPECT_GE(out.thrust, 0.811 * options.thrust_acceleration_min - 1e-8);
    EXPECT_LE(out.thrust, 0.811 * options.thrust_acceleration_max + 1e-8);
  }
}
