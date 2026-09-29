#include <px4ctrl/acados_nmpc.h>
#include <gtest/gtest.h>
#include <stdexcept>

TEST(ControllerAdapter, FeedforwardReachesOutputAndClearsOnManualAndFailure)
{
  AcadosNmpcOptions options;options.solve_time_budget_ms = 1000;
  AcadosNmpcControl controller;controller.configure(options, 1.5);
  px4ctrl::ReferencePoint flat;flat.yaw_acceleration = 0.7;
  const auto window = px4ctrl::extrapolateFlatReference(
    flat, 12, options.horizon, options.prediction_dt, options.gravity);
  Ref_State_t mode;mode.fsm_state = 3;
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
