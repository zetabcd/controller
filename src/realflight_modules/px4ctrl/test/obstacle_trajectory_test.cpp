#include <px4ctrl/obstacle_trajectory.h>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace px4ctrl;

TEST(ObstacleTrajectory, WaitsForFreshThreeDimensionalProximityAfterTakeoff)
{
  ObstacleTrajectoryOptions o;ObstacleTriggeredTrajectory t(o);
  const Eigen::Vector3d aircraft(3,4,5);
  EXPECT_FALSE(t.update(1,aircraft,aircraft,0));
  EXPECT_FALSE(t.update(3.5,aircraft,aircraft+Eigen::Vector3d(0,0,1),0));
  EXPECT_FALSE(t.update(4,aircraft,aircraft,.21));
  EXPECT_FALSE(t.update(5,aircraft,aircraft,-.01));
  EXPECT_FALSE(t.update(6,aircraft,aircraft,std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(t.update(7,aircraft,Eigen::Vector3d::Constant(NAN),0));
  EXPECT_FALSE(t.triggered());
  for(double time : {4.,100.,10000.}) {
    const auto r=t.evaluate(time);
    EXPECT_NEAR(r.position.z(),o.takeoff_height,1e-12);
    EXPECT_DOUBLE_EQ(r.position.x(),0);EXPECT_TRUE(r.velocity.isZero());
  }
  EXPECT_TRUE(t.update(10,aircraft,aircraft+Eigen::Vector3d(.5,.5,.5),.1));
  EXPECT_TRUE(t.triggered());
  EXPECT_FALSE(t.update(11,aircraft,aircraft,0));
  EXPECT_FALSE(t.update(30,aircraft,aircraft,0));
  EXPECT_DOUBLE_EQ(t.duration(),13);
  EXPECT_NEAR(t.evaluate(50).position.x(),o.move_distance,1e-12);
}

TEST(ObstacleTrajectory, WorldXIsIndependentOfHeadingAndTerminalHoldIsStationary)
{
  ObstacleTrajectoryOptions o;o.move_distance=-1.4;
  auto t=std::make_shared<ObstacleTriggeredTrajectory>(o);
  t->reset(1.5707963267948966);
  ASSERT_TRUE(t->update(8,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0));
  TrajectoryPlayer player;player.start(t,100,{2,-3,.2},0);
  const auto end=player.sample(120,16,.03).points.back();
  EXPECT_NEAR(end.position.x(),.6,1e-12);
  EXPECT_NEAR(end.position.y(),-3,1e-12);
  EXPECT_NEAR(end.position.z(),.7,1e-12);
  EXPECT_NEAR(end.yaw,1.5707963267948966,1e-12);
  EXPECT_TRUE(end.velocity.isZero());EXPECT_TRUE(end.acceleration.isZero());
  EXPECT_TRUE(end.jerk.isZero());EXPECT_TRUE(end.snap.isZero());
  t->reset(-.4);
  EXPECT_FALSE(t->triggered());
  EXPECT_NEAR(t->evaluate(20).position.x(),0,1e-12);
  EXPECT_TRUE(t->update(4,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0));
}

TEST(ObstacleTrajectory, PredictionDoesNotTriggerAndConnectionsRemainC4)
{
  auto t=std::make_shared<ObstacleTriggeredTrajectory>(ObstacleTrajectoryOptions{});
  TrajectoryPlayer player;player.start(t,100,Eigen::Vector3d::Zero(),0);
  const auto waiting=player.sample(110,16,.03);
  for(const auto &r:waiting.points) {EXPECT_DOUBLE_EQ(r.position.x(),0);}
  ASSERT_FALSE(t->triggered());
  ASSERT_TRUE(t->update(10,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0));
  const auto moving=player.sample(110,16,.03);
  EXPECT_DOUBLE_EQ(moving.points.front().position.x(),0);
  EXPECT_GT(moving.points.back().position.x(),0);
  for(double b:t->boundaries()) {
    const auto left=t->evaluate(b-1e-8),right=t->evaluate(b+1e-8);
    EXPECT_LT((left.position-right.position).norm(),1e-6);
    EXPECT_LT((left.velocity-right.velocity).norm(),1e-6);
    EXPECT_LT((left.acceleration-right.acceleration).norm(),1e-6);
    EXPECT_LT((left.jerk-right.jerk).norm(),1e-5);
    EXPECT_LT((left.snap-right.snap).norm(),1e-4);
  }
  const auto audit=auditTrajectory(*t,TrajectoryLimits{});
  EXPECT_TRUE(audit.valid)<<audit.reason;
  EXPECT_THROW(t->update(9,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0),std::invalid_argument);
}

TEST(ObstacleTrajectory, RejectsInvalidParameters)
{
  ObstacleTrajectoryOptions o;o.trigger_distance=0;
  EXPECT_THROW(ObstacleTriggeredTrajectory{ o },std::invalid_argument);
  o={};o.move_duration=0;EXPECT_THROW(ObstacleTriggeredTrajectory{ o },std::invalid_argument);
  o={};o.move_distance=NAN;EXPECT_THROW(ObstacleTriggeredTrajectory{ o },std::invalid_argument);
  o={};o.takeoff_height=-1;EXPECT_THROW(ObstacleTriggeredTrajectory{ o },std::invalid_argument);
  o={};o.pose_timeout=-1;EXPECT_THROW(ObstacleTriggeredTrajectory{ o },std::invalid_argument);
}

TEST(ObstaclePosition, RejectsStaleFutureRepeatedInvalidAndMismatchedFrames)
{
  ObstaclePosition p;
  const Eigen::Vector3d x(1,2,3);
  EXPECT_FALSE(std::isfinite(p.age(10)));
  EXPECT_FALSE(p.accept(x,9,10,"map",.2));
  EXPECT_FALSE(p.accept(x,10.1,10,"map",.2));
  EXPECT_FALSE(p.accept(Eigen::Vector3d::Constant(NAN),10,10,"map",.2));
  ASSERT_TRUE(p.accept(x,10,10.01,"map",.2));
  EXPECT_NEAR(p.age(10.1),.1,1e-12);
  EXPECT_FALSE(p.accept(x,10,10.02,"map",.2));
  EXPECT_FALSE(p.accept(x,10.02,10.02,"different",.2));
  EXPECT_GT(p.age(10.3),.2);
  EXPECT_FALSE(std::isfinite(p.age(9)));
  p.clear();EXPECT_FALSE(std::isfinite(p.age(11)));
  EXPECT_FALSE(p.accept(x,11,11,"different",.2));
  EXPECT_TRUE(p.accept(x,11,11,"map",.2));
  ObstaclePosition configured("world");
  EXPECT_FALSE(configured.accept(x,10,10,"map",.2));
  EXPECT_TRUE(configured.accept(x,10,10,"world",.2));
}
