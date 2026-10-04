#include <gtest/gtest.h>
#include <px4ctrl/external_trajectory.h>
using namespace px4ctrl;
namespace {
ReferencePoint polynomial(double t)
{
  ReferencePoint r;
  r.position={-0.8+0.2*(10*t*t*t-15*std::pow(t,4)+6*std::pow(t,5)),0,0.8};
  r.velocity={0.2*(30*t*t-60*t*t*t+30*std::pow(t,4)),0,0};
  r.acceleration={0.2*(60*t-180*t*t+120*t*t*t),0,0};
  r.jerk={0.2*(60-360*t+360*t*t),0,0};
  r.snap={0.2*(-360+720*t),0,0};return r;
}
TimedReferences samples()
{TimedReferences p;for(int i=0;i<=200;++i) {p.push_back({i/200.0,polynomial(i/200.0)});}return p;}
}
TEST(ExternalTrajectory, PreservesPolynomialAndModelAtOffGridTimes)
{
  ExternalModel model;model.drag={0.32,0.34,0.51};model.lift=0.01;
  ExternalTrajectory t(samples(),model);
  for(double time=0.0013;time<1;time+=0.0173) {
    const auto actual=t.evaluate(time), expected=externalReference(polynomial(time),model);
    EXPECT_LT((actual.position-expected.position).norm(),1e-12);
    EXPECT_LT((actual.velocity-expected.velocity).norm(),1e-10);
    EXPECT_LT((actual.acceleration-expected.acceleration).norm(),1e-8);
    EXPECT_LT(actual.attitude.angularDistance(expected.attitude),1e-8);
    EXPECT_LT((actual.body_rate-expected.body_rate).norm(),1e-5);
    EXPECT_LT((actual.body_acceleration-expected.body_acceleration).norm(),0.02);
    EXPECT_TRUE(actual.aerodynamics_included);
  }
  EXPECT_EQ(t.evaluate(2).velocity.norm(),0);
  EXPECT_EQ(t.evaluate(2).body_rate.norm(),0);
}
TEST(ExternalTrajectory, RejectsMalformedAndMovingEndpoints)
{
  ExternalModel model;auto p=samples();p[10].time=p[9].time;
  EXPECT_THROW(ExternalTrajectory(p,model),std::invalid_argument);
  p=samples();p[100].point.position.x()=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ExternalTrajectory(p,model),std::invalid_argument);
  p=samples();p.back().point.velocity.x()=0.1;
  EXPECT_THROW(ExternalTrajectory(p,model),std::invalid_argument);
}
TEST(ExternalTrajectory, WorldPlayerDoesNotReanchorToVehicle)
{
  ExternalModel model;auto t=std::make_shared<ExternalTrajectory>(samples(),model);
  TrajectoryPlayer player;player.start(t,40,Eigen::Vector3d::Zero(),0);
  const auto window=player.sample(40.2,16,0.03);
  EXPECT_EQ(window.points.size(),17u);
  EXPECT_LT((window.points[0].position-t->evaluate(0.2).position).norm(),1e-12);
  EXPECT_LT((window.points[16].position-t->evaluate(0.68).position).norm(),1e-12);
}
