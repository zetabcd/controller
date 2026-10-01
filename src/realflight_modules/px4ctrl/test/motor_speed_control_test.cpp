#include <gtest/gtest.h>
#include <px4ctrl/motor_speed_control.h>
#include <px4ctrl/motor_feedback.h>

TEST(MotorSpeedControl, IntegratesNewSamplesWithSourceDtAndKeepsFastFeedforward)
{
  MotorSpeedControl controller;
  controller.configure(0.001, 0.1, 0.25);
  EXPECT_DOUBLE_EQ(controller.update(0.3, 1000, 800, 1000000, 1, true, true), 0.3);
  EXPECT_NEAR(controller.update(0.3, 1000, 800, 1030000, 1.03, true, true), 0.306, 1e-12);
  for (int i = 1; i < 12; ++i) {
    EXPECT_NEAR(controller.update(0.4, 1000, 800, 1030000, 1.03 + i * 0.0025,
      true, true), 0.406, 1e-12);
  }
  // Arrival jitter must not replace the actual 60 ms source sampling interval.
  EXPECT_NEAR(controller.update(0.4, 1000, 800, 1090000, 1.10, true, true), 0.418, 1e-12);
}

TEST(MotorSpeedControl, LimitsIntegralAndUnwindsAtBothActuatorLimits)
{
  MotorSpeedControl controller;
  controller.configure(0.001, 0.1, 0.25);
  controller.update(0.95, 1000, 0, 1000000, 1, true, true);
  for (int i = 1; i <= 10; ++i) {
    EXPECT_DOUBLE_EQ(controller.update(0.95, 1000, 0, 1000000 + i * 100000,
      1 + i * 0.1, true, true), 1.0);
  }
  EXPECT_NEAR(controller.update(0.95, 1000, 1100, 2100000, 2.1, true, true), 0.99, 1e-12);
  controller.reset();
  controller.update(0.05, 1000, 2000, 1000000, 1, true, true);
  for (int i = 1; i <= 10; ++i) {
    EXPECT_DOUBLE_EQ(controller.update(0.05, 1000, 2000, 1000000 + i * 100000,
      1 + i * 0.1, true, true), 0.0);
  }
  EXPECT_NEAR(controller.update(0.05, 1000, 900, 2100000, 2.1, true, true), 0.01, 1e-12);
  controller.reset();
  controller.update(0.4, 1000, 0, 1000000, 1, true, true);
  EXPECT_DOUBLE_EQ(controller.update(0.4, 1000, 0, 1200000, 1.2, true, true), 0.5);
}

TEST(MotorSpeedControl, OutageDecaysCorrectionAndRecoveryDoesNotIntegrateTheGap)
{
  MotorSpeedControl controller;
  controller.configure(0.001, 0.1, 0.25);
  controller.update(0.4, 1000, 800, 1000000, 1, true, true);
  EXPECT_NEAR(controller.update(0.4, 1000, 800, 1200000, 1.2, true, true), 0.44, 1e-12);
  EXPECT_NEAR(controller.update(0.4, 1000, NAN, 1200000, 1.225, false, true), 0.43, 1e-12);
  EXPECT_NEAR(controller.update(0.4, 1000, 800, 1300000, 1.25, true, true), 0.43, 1e-12);
  EXPECT_NEAR(controller.update(0.4, 1000, 800, 5000000, 5, true, true), 0.43, 1e-12);
  EXPECT_NEAR(controller.update(0.4, 1000, 800, 1000, 5.01, true, true), 0.43, 1e-12);
  EXPECT_NEAR(controller.update(0.4, 1000, NAN, 0, 5.26, false, true), 0.4, 1e-12);
}

TEST(MotorSpeedControl, InactiveAndDisabledResetWithoutLearningFromZeroRpm)
{
  MotorSpeedControl controller;
  controller.configure(0.001, 0.1, 0.25);
  controller.update(0.4, 1000, 0, 1000000, 1, true, true);
  controller.update(0.4, 1000, 0, 1100000, 1.1, true, true);
  EXPECT_DOUBLE_EQ(controller.update(-1, 1000, 0, 1200000, 1.2, true, false), -1);
  EXPECT_DOUBLE_EQ(controller.update(0.4, 1000, 0, 1300000, 1.3, true, true), 0.4);
  controller.configure(0, 0.1, 0.25);
  EXPECT_DOUBLE_EQ(controller.update(0.4, 1000, 0, 1400000, 1.4, true, true), 0.4);
  EXPECT_DOUBLE_EQ(controller.update(0.4, 1000, 0, 1500000, 1.5, true, true), 0.4);
}

TEST(MotorSpeedControl, RejectsInvalidParameters)
{
  MotorSpeedControl controller;
  EXPECT_THROW(controller.configure(-1, 0.1, 0.25), std::invalid_argument);
  EXPECT_THROW(controller.configure(NAN, 0.1, 0.25), std::invalid_argument);
  EXPECT_THROW(controller.configure(0.001, 1.1, 0.25), std::invalid_argument);
  EXPECT_THROW(controller.configure(0.001, 0.1, 0), std::invalid_argument);
}

TEST(MotorSpeedControl, FourIndependentMotorsRejectStaticGainMismatchWithSlowTelemetry)
{
  // Synthetic motor plant only: 400 Hz actuation, 33.3 Hz ESC, 80 ms motor lag.
  // Different gains stand in for calibration/load mismatch; no voltage input.
  std::array<MotorSpeedControl, 4> controllers;
  const std::array<double, 4> gains{3400, 3600, 4000, 4200};
  std::array<double, 4> speed{}, measured{};
  uint64_t stamp = 0;
  for (auto & controller : controllers) controller.configure(0.0002, 0.1, 0.25);
  for (int k = 0; k < 6000; ++k) {
    if (k % 12 == 0) {
      stamp = 1000000 + k * 2500;
      measured = speed;
    }
    for (size_t i = 0; i < 4; ++i) {
      const double throttle = controllers[i].update(0.25, 1300, measured[i], stamp,
        k * 0.0025, true, true);
      ASSERT_GE(throttle, 0);
      ASSERT_LE(throttle, 1);
      speed[i] += (300 + gains[i] * throttle - speed[i]) * 0.0025 / 0.08;
    }
  }
  for (double motor_speed : speed) EXPECT_NEAR(motor_speed, 1300, 1.0);
}

namespace
{
px4_msgs::msg::EscStatus escMessage()
{
  px4_msgs::msg::EscStatus msg;
  msg.timestamp = 1790788980076039;  // Outer timestamp translated by DDS bridge.
  msg.esc_count = 4;
  msg.esc_online_flags = 15;
  const std::array<int, 4> slots{1, 3, 2, 0};
  for (size_t i = 0; i < 4; ++i) {
    auto & report = msg.esc[slots[i]];
    report.timestamp = 189543690 + i * 1000;  // Nested timestamps remain PX4 uptime.
    report.actuator_function = 101 + i;
    report.esc_rpm = 10000 + i * 100;
    report.esc_voltage = NAN;
  }
  return msg;
}
}

TEST(MotorFeedback, MixedClockDomainsAndBadVoltageDoNotInvalidateFreshRpm)
{
  rate_diagnostics::MotorFeedback feedback;
  feedback.configure(0.25, {}, {101, 102, 103, 104});
  feedback.update(escMessage(), 1);
  const auto sample = feedback.sample(1.014, Parameter_t{}, Eigen::Vector3d::Zero(), false);
  EXPECT_EQ(sample.schema_version, 2);
  const std::array<int8_t, 4> slots{1, 3, 2, 0};
  EXPECT_EQ(sample.esc_slot, slots);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(sample.rpm_valid[i]);
    EXPECT_FALSE(sample.electrical_valid[i]);
    EXPECT_DOUBLE_EQ(sample.motor_rpm[i], 10000 + i * 100);
    EXPECT_NEAR(sample.motor_rad[i], sample.motor_rpm[i] * 2 * M_PI / 60, 1e-9);
  }
  EXPECT_NEAR(sample.report_source_age_s[1], 0.003, 1e-8);
}

TEST(MotorFeedback, RepeatedOuterMessagesDoNotKeepFrozenMotorFresh)
{
  rate_diagnostics::MotorFeedback feedback;
  auto msg = escMessage();
  feedback.update(msg, 1);
  msg.timestamp += 300000;
  msg.esc[3].timestamp += 300000;
  feedback.update(msg, 1.3);
  auto sample = feedback.sample(1.3, Parameter_t{}, Eigen::Vector3d::Zero(), false);
  EXPECT_FALSE(sample.rpm_valid[0]);
  EXPECT_TRUE(sample.rpm_valid[1]);
  EXPECT_FALSE(sample.rpm_valid[2]);
  EXPECT_FALSE(sample.rpm_valid[3]);
  sample = feedback.sample(1.6, Parameter_t{}, Eigen::Vector3d::Zero(), false);
  for (bool valid : sample.rpm_valid) EXPECT_FALSE(valid);
}

TEST(MotorFeedback, AmbiguousMissingAndOfflineMotorsAreInvalid)
{
  rate_diagnostics::MotorFeedback feedback;
  auto msg = escMessage();
  msg.esc[3].actuator_function = 101;  // Duplicate M1, missing M2.
  msg.esc_online_flags &= ~(1u << 2);  // M3 offline.
  msg.esc[0].timestamp = 0;           // M4 has no report.
  feedback.update(msg, 1);
  const auto sample = feedback.sample(1.01, Parameter_t{}, Eigen::Vector3d::Zero(), false);
  EXPECT_EQ(sample.esc_slot[0], -2);
  EXPECT_EQ(sample.esc_slot[1], -1);
  for (bool valid : sample.rpm_valid) EXPECT_FALSE(valid);
}
