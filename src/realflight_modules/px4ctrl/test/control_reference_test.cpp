#include <px4ctrl/control_reference.h>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using namespace px4ctrl;

TEST(ControlReference, HoverAndYawDerivatives)
{
  ReferencePoint flat;
  flat.yaw = 0.4; flat.yaw_rate = 0.8; flat.yaw_acceleration = -0.3;
  const auto r = resolveReference(flat, 9.81);
  EXPECT_NEAR(r.thrust_acceleration, 9.81, 1e-12);
  EXPECT_LT((r.body_rate - Eigen::Vector3d(0, 0, 0.8)).norm(), 1e-12);
  EXPECT_LT((r.body_acceleration - Eigen::Vector3d(0, 0, -0.3)).norm(), 1e-12);
  EXPECT_TRUE(r.angular_acceleration_valid);
}

TEST(ControlReference, AnalyticFlatnessMatchesIndependentDifferentiation)
{
  ReferencePoint flat;
  flat.acceleration = {0.7, -0.2, 0.1}; flat.jerk = {0.3, 0.4, 0.2}; flat.snap = {-0.2, 0.5, 0.1};
  flat.yaw = 0.4; flat.yaw_rate = 0.6; flat.yaw_acceleration = -0.2;
  const auto sample = [&](double t) {
      auto f = flat;
      f.acceleration += t * flat.jerk + 0.5 * t * t * flat.snap;
      f.jerk += t * flat.snap;
      f.yaw += t * flat.yaw_rate + 0.5 * t * t * flat.yaw_acceleration;
      f.yaw_rate += t * flat.yaw_acceleration;
      return resolveReference(f, 9.81);
    };
  constexpr double h = 1e-5;
  const auto r = sample(0), a = sample(-h), b = sample(h);
  const auto rotation = r.attitude.toRotationMatrix();
  EXPECT_NEAR(std::atan2(rotation(1, 0), rotation(0, 0)), flat.yaw, 1e-12);
  const Eigen::Matrix3d omega = r.attitude.toRotationMatrix().transpose() *
    (b.attitude.toRotationMatrix() - a.attitude.toRotationMatrix()) / (2 * h);
  const Eigen::Vector3d numerical_rate(omega(2, 1), omega(0, 2), omega(1, 0));
  EXPECT_LT((r.body_rate - numerical_rate).norm(), 1e-8);
  EXPECT_LT((r.body_acceleration - (b.body_rate - a.body_rate) / (2 * h)).norm(), 1e-8);
}

TEST(ControlReference, FlatWindowPreservesDerivativeChainAndGrid)
{
  ReferencePoint r;
  r.velocity.x() = 1; r.acceleration.x() = 2; r.jerk.x() = 3; r.snap.x() = 4;
  const auto w = extrapolateFlatReference(r, 123, 4, 0.1, 9.81);
  ASSERT_NO_THROW(validateReferenceWindow(w, 4, 0.1));
  const double t = 0.4;
  EXPECT_NEAR(w.points.back().position.x(), t + t*t + t*t*t/2 + t*t*t*t/6, 1e-12);
  EXPECT_NEAR(w.points.back().velocity.x(), 1 + 2*t + 1.5*t*t + 4*t*t*t/6, 1e-12);
  EXPECT_NEAR(w.points.back().acceleration.x(), 2 + 3*t + 2*t*t, 1e-12);
  EXPECT_THROW(validateReferenceWindow(w, 3, 0.1), std::invalid_argument);
  EXPECT_THROW(validateReferenceWindow(w, 4, 0.2), std::invalid_argument);
}

TEST(ControlReference, MissingFeedforwardAndFrameTransform)
{
  ReferencePoint r;
  r.full_state = true;r.thrust_acceleration = 9.81;
  r.body_acceleration.setConstant(std::numeric_limits<double>::quiet_NaN());
  auto resolved = resolveReference(r, 9.81);
  EXPECT_FALSE(resolved.angular_acceleration_valid);
  EXPECT_TRUE(angularFeedforward(resolved, Eigen::Quaterniond::Identity()).isZero());
  r.angular_acceleration_valid = true;
  EXPECT_THROW(resolveReference(r, 9.81), std::invalid_argument);
  r.body_acceleration = {1, 0, 0};
  r.attitude = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  const auto actual = Eigen::Quaterniond(Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ()));
  resolved = resolveReference(r, 9.81);
  const Eigen::Vector3d expected(std::cos(1.0), std::sin(1.0), 0);
  EXPECT_LT((angularFeedforward(resolved, actual) - expected).norm(), 1e-12);
  resolved.attitude.coeffs() *= -1;
  EXPECT_LT((angularFeedforward(resolved, actual) - expected).norm(), 1e-12);
}

TEST(ControlReference, RejectsSingularAndNonfiniteReferences)
{
  ReferencePoint r;r.acceleration.z() = -9.81;
  EXPECT_THROW(resolveReference(r, 9.81), std::invalid_argument);
  r = {};r.jerk.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(resolveReference(r, 9.81), std::invalid_argument);
  r = {};r.full_state = true;r.thrust_acceleration = 9.81;r.attitude.coeffs().setZero();
  EXPECT_THROW(resolveReference(r, 9.81), std::invalid_argument);
}

TEST(ControlReference, SampledAngularAccelerationIncludesChangingAxes)
{
  ReferenceWindow w{0, 0.1, {}};
  const Eigen::Vector3d world_rate(1, 2, 0);
  for (int k = 0; k < 3; ++k) {
    ReferencePoint r;r.full_state = true;r.thrust_acceleration = 9.81;
    r.attitude = Eigen::Quaterniond(Eigen::AngleAxisd(k * 0.1, Eigen::Vector3d::UnitZ()));
    r.body_rate = r.attitude.conjugate() * world_rate;
    w.points.push_back(r);
  }
  differentiateAngularRate(w);
  for (const auto & r : w.points) {
    EXPECT_LT(r.body_acceleration.norm(), 1e-12);
  }
}

TEST(ControlReference, RejectsNonfiniteModelAndThrustDerivative)
{
  ReferenceWindow w;
  w.dt = 0.02;
  w.points.push_back(resolveReference(ReferencePoint{}, 9.805));
  auto & r = w.points.front();
  r.aerodynamics_included = true;
  r.model_horizontal_lift = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validateReferenceWindow(w, 0, w.dt), std::invalid_argument);
  r.model_horizontal_lift = 0;
  r.thrust_rate_valid = true;
  r.thrust_rate = std::numeric_limits<double>::infinity();
  EXPECT_THROW(validateReferenceWindow(w, 0, w.dt), std::invalid_argument);
}
