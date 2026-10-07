// Exercise the node's public ROS interface in one executor, without a simulator.
#define main gap_planner_entrypoint
#include "../src/planner_node.cpp"
#undef main
#include <gtest/gtest.h>
#include <thread>

TEST(PlannerFlow, LiveStartsFixedEndpointsAndFreshHomeAfterCommandExit)
{
  const char *arguments[]={"planner_flow_test","--ros-args","-p","simulation:=true",
    "-p","gate_order:=[gap_1]","-p","gates.gap_1.sim_position:=[0.0,0.3,1.7]",
    "-p","gates.gap_1.sim_rpy_deg:=[0.0,0.0,0.0]","-p","gates.gap_1.width:=2.0",
    "-p","gates.gap_1.height:=2.0","-p","planning.solve_budget:=5.0"};
  rclcpp::init(sizeof(arguments)/sizeof(arguments[0]),arguments);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  auto planner=std::make_shared<GapPlannerNode>();
  auto probe=std::make_shared<rclcpp::Node>("planner_probe",rclcpp::NodeOptions().use_global_arguments(false));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);executor.add_node(probe);
  gap_msgs::msg::ExecutionStatus state;
  state.command_mode=true;state.feedback_valid=true;state.hovering=false;
  state.origin_valid=false; // Endpoints must not depend on a ground origin.
  state.pose.orientation.w=1;
  assign(state.pose.position,Eigen::Vector3d(-7,1,3));
  auto feedback=probe->create_publisher<gap_msgs::msg::ExecutionStatus>("/gap/execution",10);
  auto timer=probe->create_wall_timer(std::chrono::milliseconds(20),[&]() {
    state.header.stamp=probe->now();feedback->publish(state);
  });
  auto configure=probe->create_service<gap_msgs::srv::ConfigureGates>("/gap/sim/configure",
    [](const std::shared_ptr<gap_msgs::srv::ConfigureGates::Request> req,
       std::shared_ptr<gap_msgs::srv::ConfigureGates::Response> res) {res->accepted=req->count==1;});
  std::vector<gap_msgs::msg::PolynomialPlan> plans;
  std::string failure;
  const auto latched=rclcpp::QoS(1).reliable().transient_local();
  auto trajectory=probe->create_subscription<gap_msgs::msg::PolynomialPlan>("/gap/polynomial",latched,
    [&](gap_msgs::msg::PolynomialPlan::ConstSharedPtr m) {plans.push_back(*m);});
  auto status=probe->create_subscription<std_msgs::msg::String>("/gap/planner_status",latched,
    [&](std_msgs::msg::String::ConstSharedPtr m) {if(m->data.find("FAILED")==0) {failure=m->data;}});
  auto client=probe->create_client<gap_msgs::srv::PlanGaps>("/gap/plan");
  auto until=[&](auto predicate,double seconds=10) {
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(std::chrono::steady_clock::now()<end) {
      executor.spin_some();if(predicate()) {return true;}
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  };
  auto settle=[&]() {
    const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(150);
    ASSERT_TRUE(until([&]() {return std::chrono::steady_clock::now()>=end;}));
  };
  auto request=[&](unsigned count) {
    auto req=std::make_shared<gap_msgs::srv::PlanGaps::Request>();req->count=count;
    auto future=client->async_send_request(req);
    if(!until([&]() {return future.wait_for(std::chrono::seconds(0))==std::future_status::ready;})) {
      throw std::runtime_error("Planner service response timed out");
    }
    return future.get();
  };
  auto checkPlan=[&](const Eigen::Vector3d &position,const Eigen::Vector3d &goal) {
    assign(state.pose.position,position);state.command_mode=true;state.hovering=true;settle();
    failure.clear();const auto previous=plans.size();
    const auto response=request(1);ASSERT_TRUE(response->accepted)<<response->message;
    ASSERT_TRUE(until([&]() {return plans.size()>previous || !failure.empty();},30));
    ASSERT_TRUE(failure.empty())<<failure;ASSERT_GT(plans.size(),previous);
    const auto &plan=plans.back();Eigen::Vector3d first,last;
    const auto offset=plan.coefficients.size()-18;const double t=plan.durations.back();
    for(int row=0;row<3;++row) {
      first[row]=plan.coefficients[row*6];last[row]=0;
      for(int col=0;col<6;++col) {last[row]+=plan.coefficients[offset+row*6+col]*std::pow(t,col);}
    }
    EXPECT_LT((first-position).norm(),1e-5);EXPECT_LT((last-goal).norm(),1e-5);
  };
  ASSERT_TRUE(until([&]() {return client->service_is_ready();}));settle();
  EXPECT_FALSE(request(1)->accepted);
  state.hovering=true;settle();EXPECT_FALSE(request(0)->accepted);
  const Eigen::Vector3d home(-2,.3,1.7),far(2,.3,1.7);
  ASSERT_NO_FATAL_FAILURE(checkPlan(home,far));
  ASSERT_NO_FATAL_FAILURE(checkPlan(far+Eigen::Vector3d(.03,-.02,.04),home));
  ASSERT_NO_FATAL_FAILURE(checkPlan(home+Eigen::Vector3d(-.02,.01,-.03),far));
  state.command_mode=false;state.hovering=false;settle();
  state.command_mode=true;settle();EXPECT_FALSE(request(1)->accepted);
  const Eigen::Vector3d new_home(-2.2,.2,1.9);
  ASSERT_NO_FATAL_FAILURE(checkPlan(new_home,new_home+Eigen::Vector3d(4,0,0)));
}
