#include <gap_planner/mocap_input.h>
#include <gtest/gtest.h>
#include <limits>
using namespace gap_planner;
namespace {
geometry_msgs::msg::PoseStamped recordedPose()
{
  // First numeric pose from flight_20261001_203447_613650_bc3a4984.
  // frame_id was omitted by the ULog recorder: this test label is synthetic.
  geometry_msgs::msg::PoseStamped m;m.header.frame_id="vrpn_world";
  m.header.stamp.sec=100;m.header.stamp.nanosec=10000000;
  m.pose.position.x=-1.2188478708267212;m.pose.position.y=-0.16532912850379944;
  m.pose.position.z=0.06636977195739746;
  m.pose.orientation.w=0.9993492960929871;m.pose.orientation.x=0.028699833899736404;
  m.pose.orientation.y=0.011320396326482296;m.pose.orientation.z=0.018685413524508476;
  return m;
}
const auto zero=Eigen::Vector3d::Zero().eval();
const auto identity=Eigen::Quaterniond::Identity();
}
TEST(GapMocap, RecordedRawPoseIsNotConvertedToAircraftNed)
{
  MocapInput input("",zero,identity,0.3);auto m=recordedPose();MocapPose out;std::string why;
  ASSERT_TRUE(input.accept(m,100.02,-1,zero,identity,out,why))<<why;
  EXPECT_EQ(input.frame(),"vrpn_world");EXPECT_TRUE(input.frameLocked());
  EXPECT_NEAR(out.center.y(),m.pose.position.y,1e-12);
  EXPECT_NEAR(out.center.z(),m.pose.position.z,1e-12);
  Eigen::Quaterniond q(m.pose.orientation.w,m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z);
  EXPECT_LT(out.rotation.angularDistance(q.normalized()),1e-12);
}
TEST(GapMocap, RigidBodyOffsetThenWorldCalibrationThenOpeningRotation)
{
  const Eigen::Quaterniond world(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond opening(Eigen::AngleAxisd(M_PI/3,Eigen::Vector3d::UnitX()));
  MocapInput input("world",{10,20,30},world,0.3);auto m=recordedPose();
  m.header.frame_id="world";m.pose.position.x=1;m.pose.position.y=2;m.pose.position.z=3;
  m.pose.orientation.w=std::sqrt(0.5);m.pose.orientation.x=0;m.pose.orientation.y=0;
  m.pose.orientation.z=std::sqrt(0.5);
  MocapPose out;std::string why;
  ASSERT_TRUE(input.accept(m,100.02,-1,{1,0,0},opening,out,why))<<why;
  EXPECT_LT((out.center-Eigen::Vector3d(7,21,33)).norm(),1e-12);
  const Eigen::Quaterniond expected=world*world*opening;
  EXPECT_LT(out.rotation.angularDistance(expected),1e-12);
}
TEST(GapMocap, InvalidDataCannotLockFrameOrReplaceLastPose)
{
  MocapInput input("",zero,identity,0.3);auto m=recordedPose();MocapPose out{7,{8,9,10},identity};
  std::string why;
  EXPECT_FALSE(input.accept(m,101,-1,zero,identity,out,why));
  EXPECT_FALSE(input.frameLocked());EXPECT_EQ(out.stamp,7);
  m.pose.orientation.w=0;m.pose.orientation.x=0;m.pose.orientation.y=0;m.pose.orientation.z=0;
  EXPECT_FALSE(input.accept(m,100.02,-1,zero,identity,out,why));EXPECT_FALSE(input.frameLocked());
  m=recordedPose();m.pose.position.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(input.accept(m,100.02,-1,zero,identity,out,why));EXPECT_FALSE(input.frameLocked());
  m=recordedPose();
  EXPECT_FALSE(input.accept(m,99.9,-1,zero,identity,out,why));
  EXPECT_FALSE(input.accept(m,100.02,100.01,zero,identity,out,why));
  m.header.stamp.nanosec=1000000000u;
  EXPECT_FALSE(input.accept(m,100.02,-1,zero,identity,out,why));
  EXPECT_FALSE(input.frameLocked());EXPECT_EQ(out.stamp,7);
}
TEST(GapMocap, AllRigidBodiesUseSameLockedFrameIncludingEmptyLabel)
{
  MocapInput input("",zero,identity,0.3);auto m=recordedPose();MocapPose out;std::string why;
  m.header.frame_id="";
  ASSERT_TRUE(input.accept(m,100.02,-1,zero,identity,out,why));
  EXPECT_TRUE(input.frameLocked());EXPECT_EQ(input.frame(),"");
  m.header.frame_id="different";
  EXPECT_FALSE(input.accept(m,100.02,-1,zero,identity,out,why));EXPECT_NE(why.find("frame mismatch"),std::string::npos);
  MocapInput explicit_frame("configured_world",zero,identity,0.3);
  EXPECT_FALSE(explicit_frame.accept(m,100.02,-1,zero,identity,out,why));
  m.header.frame_id="configured_world";
  EXPECT_TRUE(explicit_frame.accept(m,100.02,-1,zero,identity,out,why));
}
