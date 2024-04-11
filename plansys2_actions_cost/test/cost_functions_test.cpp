// Copyright 2024 National Council of Research of Italy (CNR) - Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "plansys2_actions_cost/cost_functions/path_length.hpp"
#include "plansys2_actions_cost/cost_functions/path_smoothness.hpp"
#include "plansys2_actions_cost/cost_functions/path_cost.hpp"

#include "plansys2_actions_cost/move_action_cost_length.hpp"

#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Test case for path_length
TEST(CostFunctionTest, PathLengthTest)
{
  plansys2_actions_cost::PathLength path_length;

  nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);

  geometry_msgs::msg::PoseStamped pose1, pose2;
  pose1.pose.position.x = 0.0;
  pose1.pose.position.y = 0.0;
  pose2.pose.position.x = 1.0;
  pose2.pose.position.y = 1.0;
  path_ptr->poses.push_back(pose1);
  path_ptr->poses.push_back(pose2);

  double expected_length = sqrt(2.0);
  auto path_length_cost_function = path_length.args_binder(std::ref(path_ptr));

  double actual_length = path_length_cost_function()->nominal_cost;

  EXPECT_DOUBLE_EQ(actual_length, expected_length);
}
// Test case for path_length with multiple poses
TEST(CostFunctionTest, PathLengthMultipleTest)
{
  plansys2_actions_cost::PathLength path_length;

  nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);

  geometry_msgs::msg::PoseStamped pose1, pose2, pose3, pose4;
  pose1.pose.position.x = 0.0;
  pose1.pose.position.y = 0.0;
  pose2.pose.position.x = 1.0;
  pose2.pose.position.y = 1.0;
  pose3.pose.position.x = 2.0;
  pose3.pose.position.y = 1.0;
  pose4.pose.position.x = 3.0;
  pose4.pose.position.y = 0.0;

  path_ptr->poses.push_back(pose1);
  path_ptr->poses.push_back(pose2);
  path_ptr->poses.push_back(pose3);
  path_ptr->poses.push_back(pose4);

  double expected_length = sqrt(2.0) + sqrt(2.0) + 1.0;

  auto path_length_cost_function = path_length.args_binder(std::ref(path_ptr));

  double actual_length = path_length_cost_function()->nominal_cost;

  EXPECT_DOUBLE_EQ(actual_length, expected_length);
}
// Test case for path_smoothness
TEST(CostFunctionTest, PathSmoothnessTest)
{
  plansys2_actions_cost::PathSmoothness path_smoothness;
  nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);

  path_ptr->poses.resize(2);

  // Rotation of 10° around the z axis
  path_ptr->poses[0].pose.position.x = 1.0;
  path_ptr->poses[0].pose.position.y = 0.0;
  path_ptr->poses[0].pose.orientation.z = 0.08715574;
  path_ptr->poses[0].pose.orientation.w = 0.9961947;

  // Rotation of 30° around the z axis
  path_ptr->poses[1].pose.position.x = 2.0;
  path_ptr->poses[1].pose.position.y = 0.0;
  path_ptr->poses[1].pose.orientation.z = 0.25881905;
  path_ptr->poses[1].pose.orientation.w = 0.96592583;


  double expected_smoothness = 20.0 * (M_PI / 180.0);

  auto path_smoothness_cost_function = path_smoothness.args_binder(std::ref(path_ptr));

  double actual_smoothness = path_smoothness_cost_function()->nominal_cost;

  double tolerance = 1e-5;
  EXPECT_NEAR(actual_smoothness, expected_smoothness, tolerance);
}

// Test case for path_cost
TEST(CostFunctionTest, PathCostTest)
{
  nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);
  path_ptr->poses.resize(2);


  path_ptr->poses[0].pose.position.x = 1.0;
  path_ptr->poses[0].pose.position.y = 0.0;

  path_ptr->poses[1].pose.position.x = 2.0;
  path_ptr->poses[1].pose.position.y = 0.0;

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");

  // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  auto costmap_subscriber = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    node->shared_from_this(),
    "/fake_cost_map_raw");

  unsigned char default_value = 100;
  auto costmap_to_send = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
  for (unsigned int y = 0; y < costmap_to_send->getSizeInCellsY(); y++) {
    for (unsigned int x = 0; x < costmap_to_send->getSizeInCellsX(); x++) {
      costmap_to_send->setCost(x, y, default_value);
    }
  }
  auto costmap_publisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node->shared_from_this(), costmap_to_send.get(), "", "/fake_cost_map", true);

  costmap_publisher->on_activate();
  costmap_publisher->publishCostmap();

  rclcpp::spin_some(node->get_node_base_interface());

  auto costmap = costmap_subscriber->getCostmap();

  double lambda_1 = 1;
  double lambda_2 = 0.5;
  double action_cost_lambda_1 = std::numeric_limits<double>::infinity();
  double action_cost_lambda_2 = std::numeric_limits<double>::infinity();
  if (costmap) {
    std::cerr << costmap->getSizeInCellsX() << std::endl;
    RCLCPP_INFO(node->get_logger(), "Costmap available");

    plansys2_actions_cost::PathCost path_cost;

    std::cerr << &path_cost << std::endl;
    std::cerr << &path_cost << std::endl;

    auto path_cost_function_lambda_1 = path_cost.args_binder(
      std::ref(path_ptr), std::ref(
        costmap), std::ref(lambda_1));
    auto path_cost_function_lambda_2 = path_cost.args_binder(
      std::ref(path_ptr), std::ref(
        costmap), std::ref(lambda_2));

    action_cost_lambda_1 = path_cost_function_lambda_1()->nominal_cost;
    action_cost_lambda_2 = path_cost_function_lambda_2()->nominal_cost;
  } else {
    RCLCPP_INFO(node->get_logger(), "Costmap not available");
  }
  std::cerr << default_value * path_ptr->poses.size() << std::endl;

  ASSERT_NEAR(action_cost_lambda_1, default_value * path_ptr->poses.size(), 1e-5);
  ASSERT_NEAR(
    action_cost_lambda_2, default_value * 1 + default_value * std::pow(lambda_2, 1),
    1e-5);
}

class ComputePathToPoseActionServer : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

  explicit ComputePathToPoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("compute_path_to_pose", options)
  {
    std::cerr << "Creating action server" << std::endl;
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this,
      "/compute_path_to_pose",
      std::bind(&ComputePathToPoseActionServer::handle_goal, this, _1, _2),
      std::bind(&ComputePathToPoseActionServer::handle_cancel, this, _1),
      std::bind(&ComputePathToPoseActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    std::cerr << "Received goal request" << std::endl;
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    std::cerr << "Received request to cancel goal" << std::endl;
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    std::cerr << "Handle accepted" << std::endl;
    using namespace std::placeholders;
    
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ComputePathToPoseActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    std::cerr << "Received goal request" << std::endl;
    const auto goal = goal_handle->get_goal();

    auto result = std::make_shared<ComputePathToPose::Result>();
    result->path = generate_path(goal->start, goal->goal); // Generate a path from start to goal
    goal_handle->succeed(result);
    std::cerr << "Succeeding goal" << std::endl;
    return;
  }
  nav_msgs::msg::Path generate_path(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal)
  {
      nav_msgs::msg::Path path;
      path.header.stamp = this->now();
      path.header.frame_id = "map"; // Assuming the path is in the map frame

      // Simply add the start and goal poses to the path
      path.poses.push_back(start);
      path.poses.push_back(goal);

      return path;
  }

};  // class 

TEST(MoveActionCostTest, MoveActionCostTestLength)
{
  using namespace std::chrono_literals;
  auto test_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto action_executor_client = plansys2::ActionExecutorClient::make_shared("fake_action_executor_client", std::chrono::milliseconds(100));
  action_executor_client->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  action_executor_client->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);


  auto compute_path_action_node = std::make_shared<ComputePathToPoseActionServer>();
  auto start_conf_pub = test_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10);
  bool received_message = false; 
  auto action_result_sub = test_node->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "/actions_hub", 10, [&](const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
      std::cerr << "Action result received" << std::endl;
      ASSERT_EQ(msg->action, "move");
      received_message = true;
    });
  auto msg_test = std::make_shared<plansys2_msgs::msg::ActionExecution>();
  msg_test->action = "move";
  auto move_action_cost = std::make_shared<plansys2_actions_cost::MoveActionCostLength>();
  std::cerr << "Ide: " << action_executor_client.get() << std::endl;
  move_action_cost->initialize(action_executor_client);
  std::cerr << "Ide: " << action_executor_client.get() << std::endl;

  // // Start pose
  geometry_msgs::msg::PoseWithCovarianceStamped start_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
  start_pose.pose.pose.position.x = 0.0;
  start_pose.pose.pose.position.y = 0.0;

  // // Goal pose
  geometry_msgs::msg::PoseStamped goal_pose = geometry_msgs::msg::PoseStamped();
  goal_pose.pose.position.x = 1.0;
  goal_pose.pose.position.y = 1.0;

  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);
  exe.add_node(test_node->get_node_base_interface());
  exe.add_node(compute_path_action_node->get_node_base_interface());
  exe.add_node(action_executor_client->get_node_base_interface());

  // while(rclcpp::ok()){
  start_conf_pub->publish(start_pose);
  // rclcpp::spin_some();
  std::cerr << "Publishing and calling move_action_cost" << std::endl;
  RCLCPP_DEBUG(test_node->get_logger(), "Publishing and calling move_action_cost");
  move_action_cost->compute_action_cost(goal_pose, msg_test);
  
  std::cerr << "After pub and calling move_action_cost" << std::endl;
    
  auto start = test_node->now();
  auto rate = rclcpp::Rate(1);
  while (rclcpp::ok() && (test_node->now() - start) < 10s)  {
    exe.spin_some();
    // if (received_message) {
    //   break;
    // }
    rate.sleep();
  }
  ASSERT_TRUE(received_message);

}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
