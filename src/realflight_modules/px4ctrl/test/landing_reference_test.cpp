#include <gtest/gtest.h>
#include <px4ctrl/landing_reference.h>

using px4ctrl::LandingReference;

TEST(LandingReference, OnlyAnExplicitRequestActivatesLanding)
{
  LandingReference landing;
  landing.update(.01, {2, 3, 1}, {0, 0, 0}, true, true, false);
  EXPECT_FALSE(landing.active());
  landing.request();
  EXPECT_TRUE(landing.requested());
  landing.start({2, 3, 1}, .2);
  EXPECT_EQ(landing.phase(), LandingReference::Phase::Settling);
  landing.cancel();
  EXPECT_FALSE(landing.active());
}

TEST(LandingReference, WaitsForContinuousUprightSlowFlightBeforeDescending)
{
  LandingReference landing;
  landing.start({2, 3, 1}, .2);
  for (int i = 0; i < 100; ++i) {
    landing.update(.01, {2, 3, 1}, {1, 0, 0}, true, true, false);
  }
  for (int i = 0; i < 100; ++i) {
    landing.update(.01, {2, 3, 1}, {0, 0, 0}, true, false, false);
  }
  EXPECT_DOUBLE_EQ(landing.position().z(), 1.0);
  for (int i = 0; i < 30; ++i) landing.update(.01, {2, 3, 1}, {0, 0, 0}, true, true, false);
  landing.update(.01, {2, 3, 1}, {0, 0, 0}, false, true, false);
  for (int i = 0; i < 30; ++i) landing.update(.01, {2, 3, 1}, {0, 0, 0}, true, true, false);
  EXPECT_EQ(landing.phase(), LandingReference::Phase::Settling);
  for (int i = 0; i < 25; ++i) landing.update(.01, {2, 3, 1}, {0, 0, 0}, true, true, false);
  EXPECT_LT(landing.position().z(), 1.0);
}

TEST(LandingReference, DescendsAtBoundedSpeedAndStopsReferenceAtGroundFloor)
{
  LandingReference landing;
  landing.start({2, -3, 2}, .2);
  for (int i = 0; i < 4000; ++i) {
    const auto p = landing.position();
    landing.update(.01, p, landing.velocity(), true, true, false);
    EXPECT_DOUBLE_EQ(landing.position().x(), 2.0);
    EXPECT_DOUBLE_EQ(landing.position().y(), -3.0);
    EXPECT_LE(landing.position().z(), p.z());
    EXPECT_GE(landing.position().z(), .1 - 1e-12);
    EXPECT_GE(landing.velocity().z(), -.3 - 1e-12);
    if (p.z() < .4) EXPECT_GE(landing.velocity().z(), -.1 - 1e-12);
  }
  EXPECT_NEAR(landing.position().z(), .1, 1e-9);
  EXPECT_DOUBLE_EQ(landing.velocity().norm(), 0.0);
  EXPECT_FALSE(landing.landed()); // At ground without detector is insufficient.
}

TEST(LandingReference, RequiresFreshLandedConfirmationNearGround)
{
  LandingReference landing;
  landing.start({2, 3, 1}, 0);
  for (int i = 0; i < 200; ++i) landing.update(.01, {2, 3, 1}, {0, 0, 0}, true, true, true);
  EXPECT_FALSE(landing.landed()); // Even a landed flag must not disarm in the air.
  for (int i = 0; i < 30; ++i) landing.update(.01, {2, 3, 0}, {0, 0, 0}, true, true, true);
  landing.update(.01, {2, 3, 0}, {0, 0, 0}, true, true, false);
  for (int i = 0; i < 30; ++i) landing.update(.01, {2, 3, 0}, {0, 0, 0}, true, true, true);
  EXPECT_FALSE(landing.landed());
  for (int i = 0; i < 25; ++i) landing.update(.01, {2, 3, 0}, {0, 0, 0}, true, true, true);
  EXPECT_TRUE(landing.landed());
  EXPECT_DOUBLE_EQ(landing.velocity().norm(), 0.0);
}

TEST(LandingReference, CannotConfirmTouchdownWhileMovingOrTilted)
{
  LandingReference landing;
  landing.start({0, 0, .2}, 0);
  for (int i = 0; i < 60; ++i) landing.update(.01, {0, 0, .2}, {0, 0, 0}, true, true, false);
  for (int i = 0; i < 100; ++i) landing.update(.01, {0, 0, 0}, {0, 0, -.4}, true, true, true);
  for (int i = 0; i < 100; ++i) landing.update(.01, {0, 0, 0}, {0, 0, 0}, true, false, true);
  EXPECT_FALSE(landing.landed());
}

TEST(LandingReference, InvalidFeedbackPausesDescentAndCancellationCanRestart)
{
  LandingReference landing;
  landing.start({2, 3, 1}, .2);
  for (int i = 0; i < 100; ++i) landing.update(.01, {2, 3, 1}, {0, 0, 0}, true, true, false);
  const auto held = landing.position();
  for (int i = 0; i < 100; ++i) landing.update(.01, {2, 3, 1}, {0, 0, 0}, false, true, false);
  EXPECT_TRUE(landing.position().isApprox(held));
  landing.cancel();
  landing.request();
  landing.start({-1, 4, 2}, .3);
  EXPECT_TRUE(landing.position().isApprox(Eigen::Vector3d(-1, 4, 2)));
  EXPECT_EQ(landing.phase(), LandingReference::Phase::Settling);
}
