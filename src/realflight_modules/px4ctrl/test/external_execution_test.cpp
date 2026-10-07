#include <px4ctrl/external_execution.h>
#include <gtest/gtest.h>

class ExternalExecutionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node=std::make_shared<rclcpp::Node>("external_execution_test");
    node->declare_parameter("trajectory.takeoff_height",.6);
    node->declare_parameter("trajectory.takeoff_duration",3.0);
    node->declare_parameter("trajectory.settle_duration",.5);
    execution=std::make_unique<px4ctrl::ExternalExecution>(
      *node,px4ctrl::TrajectoryLimits{},px4ctrl::ExternalModel{},true);
    start.position={-1.5,-.9,.25};start.yaw=.6;
    yaw=Eigen::Quaterniond(Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitZ()));
    poll(10,start.position);
  }
  void poll(double now,const Eigen::Vector3d &p,bool valid=true,
    Eigen::Vector3d velocity=Eigen::Vector3d::Zero())
  {execution->poll(now,true,false,valid,p,velocity,yaw,true,{-1.5,-.9,.05});}
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<px4ctrl::ExternalExecution> execution;
  px4ctrl::ReferencePoint start;
  Eigen::Quaterniond yaw;
};

TEST_F(ExternalExecutionTest, PreparationUsesSharedRelativeHeightAndRetainsEntryXY)
{
  auto source=execution->beginCommand(10,start);
  ASSERT_TRUE(source);ASSERT_TRUE(execution->active());
  EXPECT_LT((source->evaluate(0).position-start.position).norm(),1e-12);
  EXPECT_LT((source->evaluate(3).position-Eigen::Vector3d(-1.5,-.9,.85)).norm(),1e-12);
  std::string reason;
  EXPECT_FALSE(execution->ready(reason));EXPECT_NE(reason.find("preparing"),std::string::npos);
  auto trigger=node->create_client<std_srvs::srv::Trigger>("/gap/start");
  ASSERT_TRUE(trigger->wait_for_service(std::chrono::seconds(1)));
  auto requested=trigger->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(rclcpp::spin_until_future_complete(node,requested,std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto start_response=requested.get();
  EXPECT_FALSE(start_response->success);
  EXPECT_NE(start_response->message.find("preparing"),std::string::npos);
  auto upload=node->create_client<gap_msgs::srv::UploadTrajectory>("/gap/upload");
  ASSERT_TRUE(upload->wait_for_service(std::chrono::seconds(1)));
  auto uploaded=upload->async_send_request(std::make_shared<gap_msgs::srv::UploadTrajectory::Request>());
  ASSERT_EQ(rclcpp::spin_until_future_complete(node,uploaded,std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto upload_response=uploaded.get();
  EXPECT_FALSE(upload_response->accepted);
  EXPECT_NE(upload_response->message.find("preparing"),std::string::npos);
  EXPECT_FALSE(execution->requestStart());
  EXPECT_FALSE(execution->finished(20)); // Elapsed time alone is insufficient.
}

TEST_F(ExternalExecutionTest, CompletionRequiresContinuousFreshStableFeedback)
{
  auto source=execution->beginCommand(10,start);
  const auto end=source->evaluate(source->duration()).position;
  poll(12.9,end);EXPECT_FALSE(execution->finished(12.9));
  poll(13,end);EXPECT_FALSE(execution->finished(13.4));
  poll(13.4,end,false);EXPECT_FALSE(execution->finished(14));
  poll(14,end);poll(14.3,end,true,{.2,0,0});
  poll(14.5,end);poll(15.01,end);
  EXPECT_TRUE(execution->finished(15.01));
  yaw=Eigen::Quaterniond(Eigen::AngleAxisd(1.0,Eigen::Vector3d::UnitZ()));
  poll(15.1,end);EXPECT_FALSE(execution->finished(16));
  yaw=Eigen::Quaterniond(Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitZ()));
  poll(16,end+Eigen::Vector3d(.2,0,0));EXPECT_FALSE(execution->finished(17));
  poll(17,end);poll(17.51,end);EXPECT_TRUE(execution->finished(17.51));
  execution->complete();EXPECT_FALSE(execution->active());
  EXPECT_FALSE(execution->finished(18));
  EXPECT_FALSE(execution->requestStart());
}

TEST_F(ExternalExecutionTest, StopCancelsPreparationAndReentryStartsFromCurrentState)
{
  execution->beginCommand(10,start);
  execution->stop();EXPECT_FALSE(execution->active());EXPECT_FALSE(execution->finished(30));
  start.position={2,3,.45};start.velocity={.04,0,.03};
  poll(20,start.position,true,start.velocity);
  const auto source=execution->beginCommand(20,start);
  ASSERT_TRUE(source);
  EXPECT_LT((source->evaluate(0).position-start.position).norm(),1e-12);
  EXPECT_LT((source->evaluate(0).velocity-start.velocity).norm(),1e-12);
  EXPECT_LT((source->evaluate(3).position-Eigen::Vector3d(2,3,1.05)).norm(),1e-12);
}

TEST_F(ExternalExecutionTest, EveryNewCommandEntryUsesItsOwnStartingHeight)
{
  start.position={2,3,1.05};poll(10,start.position);
  auto source=execution->beginCommand(10,start);
  ASSERT_TRUE(source);
  EXPECT_NEAR(source->evaluate(3).position.z(),1.65,1e-12);
  execution->stop();
  start.position.z()=1.65;poll(20,start.position);
  source=execution->beginCommand(20,start);
  ASSERT_TRUE(source);
  EXPECT_NEAR(source->evaluate(3).position.z(),2.25,1e-12);
}

TEST_F(ExternalExecutionTest, ZeroHeightStillBrakesAndWaitsForStableFeedback)
{
  execution.reset();node.reset();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("trajectory.takeoff_height",0.0)});
  node=std::make_shared<rclcpp::Node>("external_zero_height_test",options);
  node->declare_parameter("trajectory.takeoff_height",.6);
  node->declare_parameter("trajectory.takeoff_duration",3.0);
  node->declare_parameter("trajectory.settle_duration",.5);
  execution=std::make_unique<px4ctrl::ExternalExecution>(
    *node,px4ctrl::TrajectoryLimits{},px4ctrl::ExternalModel{},true);
  start.velocity={.04,0,.03};poll(10,start.position,true,start.velocity);
  const auto source=execution->beginCommand(10,start);
  ASSERT_TRUE(source);
  EXPECT_LT((source->evaluate(0).velocity-start.velocity).norm(),1e-12);
  EXPECT_LT((source->evaluate(3).position-start.position).norm(),1e-12);
  poll(13,start.position);EXPECT_FALSE(execution->finished(13));
  poll(13.51,start.position);EXPECT_TRUE(execution->finished(13.51));
}
