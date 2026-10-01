#include <gtest/gtest.h>
#include <px4ctrl/takeoff_origin.h>
#include <limits>

TEST(TakeoffOrigin, UsesLatestGroundPositionIncludingHeightOffset)
{
  px4ctrl::TakeoffOrigin origin;
  origin.update(false, true, {0, 0, 0});
  origin.update(false, true, {2, -3, 0.2});
  origin.update(true, true, {2.1, -3.1, 0.4});
  EXPECT_TRUE(origin.enterHover({2.1, -3.1, 0.4}).isApprox(Eigen::Vector3d(2, -3, 0.4)));
}

TEST(TakeoffOrigin, FlightMotionAndModeReentryDoNotMoveOriginOrAddHeight)
{
  px4ctrl::TakeoffOrigin origin;
  origin.update(false, true, {2, 3, 0});
  origin.enterHover({2, 3, 0});
  origin.update(true, true, {4, 5, 2});
  EXPECT_TRUE(origin.enterHover({4, 5, 2}).isApprox(Eigen::Vector3d(4, 5, 2)));
  EXPECT_TRUE(origin.position().isApprox(Eigen::Vector3d(2, 3, 0)));
  EXPECT_FALSE(origin.takeoffPending());
}

TEST(TakeoffOrigin, DisarmAndRelocationStartANewFlight)
{
  px4ctrl::TakeoffOrigin origin;
  origin.update(false, true, {2, 3, 0});
  origin.update(true, true, {2, 3, 0});
  origin.enterHover({2, 3, 0});
  origin.update(false, true, {-3, 1, -0.1});
  origin.update(true, true, {-3, 1, 0});
  EXPECT_TRUE(origin.enterHover({-3, 1, 0}).isApprox(Eigen::Vector3d(-3, 1, 0.1)));
}

TEST(TakeoffOrigin, HoverSelectedBeforeArmingDoesNotRepeatTakeoffLater)
{
  px4ctrl::TakeoffOrigin origin;
  origin.update(false, true, {2, 3, 0});
  origin.enterHover({2, 3, 0});
  origin.update(false, true, {2, 3, 0});
  origin.update(true, true, {2, 3, 0});
  EXPECT_TRUE(origin.enterHover({4, 5, 2}).isApprox(Eigen::Vector3d(4, 5, 2)));
}

TEST(TakeoffOrigin, CannotInventGroundOriginWhenStartedInAir)
{
  px4ctrl::TakeoffOrigin origin;
  origin.update(true, true, {2, 3, 4});
  EXPECT_FALSE(origin.valid());
  EXPECT_THROW(origin.enterHover({2, 3, 4}), std::invalid_argument);
}

TEST(TakeoffOrigin, InvalidGroundPositionCannotReuseOldFlightOrigin)
{
  px4ctrl::TakeoffOrigin origin;
  origin.update(false, true, {2, 3, 0});
  origin.update(false, false, {0, 0, 0});
  EXPECT_FALSE(origin.valid());
  origin.update(false, true, {std::numeric_limits<double>::quiet_NaN(), 0, 0});
  EXPECT_FALSE(origin.valid());
  EXPECT_THROW(origin.enterHover({0, 0, 0}), std::invalid_argument);
}
