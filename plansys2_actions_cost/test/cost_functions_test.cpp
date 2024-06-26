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
#include "plansys2_actions_cost/move_action_cost_smoothness.hpp"
#include "plansys2_actions_cost/move_action_cost_map.hpp"

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
class CostmapUtils
{
public:
  CostmapUtils(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::string costmap_topic_name,
    unsigned char default_value = 100)
  : node_(node)
  {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
    for (unsigned int y = 0; y < costmap_->getSizeInCellsY(); y++) {
      for (unsigned int x = 0; x < costmap_->getSizeInCellsX(); x++) {
        costmap_->setCost(x, y, default_value);
      }
    }

    costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
      node_->shared_from_this(), costmap_.get(), "", costmap_topic_name, true);

    costmap_pub_->on_activate();
  }
  void publish_costmap()
  {
    costmap_pub_->publishCostmap();
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
};
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

  explicit ComputePathToPoseActionServer(
    const
    rclcpp::NodeOptions & options = rclcpp::NodeOptions())
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
    result->path = generate_path(goal->start, goal->goal);  // Generate a path from start to goal
    goal_handle->succeed(result);
    // Print path points
    std::cerr << "Start: " << goal->start.pose.position.x << " " << goal->start.pose.position.y <<
      std::endl;
    std::cerr << "Goal: " << goal->goal.pose.position.x << " " << goal->goal.pose.position.y <<
      std::endl;
    std::cerr << "Succeeding goal" << std::endl;
    return;
  }
  nav_msgs::msg::Path generate_path(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "map";   // Assuming the path is in the map frame

    // Simply add the start and goal poses to the path
    path.poses.push_back(start);
    path.poses.push_back(goal);

    return path;
  }
};  // class

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    using std::placeholders::_1;
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
    action_executor_client_ = plansys2::ActionExecutorClient::make_shared(
      "fake_action_executor_client", std::chrono::milliseconds(100));
    compute_path_action_server_ = std::make_shared<ComputePathToPoseActionServer>();

    start_conf_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    action_result_sub_ = node_->create_subscription<plansys2_msgs::msg::ActionExecution>(
      "/actions_hub", 10, std::bind(&TestNode::action_result_callback, this, _1));

    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(compute_path_action_server_->get_node_base_interface());
    executor_.add_node(action_executor_client_->get_node_base_interface());
    activate_nodes();
  }
  bool is_message_received() {return received_message_;}  // action response received
  plansys2_msgs::msg::ActionExecution::SharedPtr get_last_message() {return last_message_;}

  void action_result_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
  {
    received_message_ = true;
    last_message_ = msg;
  }
  void execute()
  {
    auto start = node_->now();
    auto rate = rclcpp::Rate(1);
    using namespace std::chrono_literals;
    while (rclcpp::ok() && (node_->now() - start) < 10s) {
      executor_.spin_some();
      rate.sleep();
    }
  }
  void initialize_move_action_cost(
    const std::shared_ptr<plansys2_actions_cost::MoveActionCostBase> move_action_cost)
  {
    move_action_cost_ = move_action_cost;
    move_action_cost_->initialize(action_executor_client_);
  }

  void call_action_cost(
    geometry_msgs::msg::PoseStamped goal_pose,
    plansys2_msgs::msg::ActionExecution::SharedPtr msg_test)
  {
    move_action_cost_->compute_action_cost(goal_pose, msg_test);
  }
  void publish_start_pose(geometry_msgs::msg::PoseWithCovarianceStamped start_pose)
  {
    start_conf_pub_->publish(start_pose);
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  get_publish_start_pose()
  {
    return start_conf_pub_;
  }
  plansys2::ActionExecutorClient::Ptr get_action_executor_client()
  {
    return action_executor_client_;
  }
  ~TestNode()
  {
    start_conf_pub_.reset();
  }
  // void set_move_action_cost
  // (const std::shared_ptr<plansys2_actions_cost::MoveActionCostBase> move_action_cost)
  // {
  //   move_action_cost_ = move_action_cost;
  //   move_action_cost_->initialize(action_executor_client_);
  // }
  geometry_msgs::msg::PoseStamped get_current_pose_in_move_action_cost()
  {
    return move_action_cost_->get_current_pose();
  }
  void spin_all_nodes()
  {
    executor_.spin_some();
  }
  void add_node_to_executor(rclcpp::Node::SharedPtr node)
  {
    executor_.add_node(node->get_node_base_interface());
  }
  void add_node_to_executor(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  {
    executor_.add_node(node->get_node_base_interface());
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor executor_;

  plansys2::ActionExecutorClient::Ptr action_executor_client_;
  ComputePathToPoseActionServer::SharedPtr compute_path_action_server_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_result_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_conf_pub_;

  bool received_message_ = false;
  plansys2_msgs::msg::ActionExecution::SharedPtr last_message_;

  void activate_nodes()
  {
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    action_executor_client_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    // The activation is commented since in real plansys integration the action executor
    // client is activated by the action executor node
    // when the action has to be executed
    // action_executor_client_->trigger_transition(
    // lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }
  std::shared_ptr<plansys2_actions_cost::MoveActionCostBase> move_action_cost_;
};

TEST_F(TestNode, MoveActionCostTestLength)
{
  auto msg_test = std::make_shared<plansys2_msgs::msg::ActionExecution>();
  msg_test->action = "move";
  auto move_action_cost = std::make_shared<plansys2_actions_cost::MoveActionCostLength>();

  // Start pose
  geometry_msgs::msg::PoseWithCovarianceStamped start_pose =
    geometry_msgs::msg::PoseWithCovarianceStamped();
  start_pose.pose.pose.position.x = 1.0;
  start_pose.pose.pose.position.y = 1.0;

  // Goal pose
  geometry_msgs::msg::PoseStamped goal_pose = geometry_msgs::msg::PoseStamped();
  goal_pose.pose.position.x = 2.0;
  goal_pose.pose.position.y = 2.0;

  initialize_move_action_cost(move_action_cost);
  publish_start_pose(start_pose);
  spin_all_nodes();

  // Measure the time nedded to call action cost
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  call_action_cost(goal_pose, msg_test);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cerr << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(
    end - begin).count() << "[µs]" << std::endl;

  execute();

  ASSERT_TRUE(is_message_received());

  auto current_pose = get_current_pose_in_move_action_cost();
  std::cerr << "Stored pose: " << current_pose.pose.position.x << " " <<
    current_pose.pose.position.y << std::endl;
  std::cerr << "Start pose: " << start_pose.pose.pose.position.x << " " <<
    start_pose.pose.pose.position.y << std::endl;

  ASSERT_DOUBLE_EQ(current_pose.pose.position.x, start_pose.pose.pose.position.x);
  ASSERT_DOUBLE_EQ(current_pose.pose.position.y, start_pose.pose.pose.position.y);
  auto last_message = get_last_message();
  ASSERT_EQ(last_message->action, "move");
  double expected_length = sqrt(2.0);
  ASSERT_DOUBLE_EQ((last_message->action_cost).nominal_cost, expected_length);
}

TEST_F(TestNode, MoveActionCostTestSmoothness)
{
  auto msg_test = std::make_shared<plansys2_msgs::msg::ActionExecution>();
  msg_test->action = "move";
  auto move_action_cost = std::make_shared<plansys2_actions_cost::MoveActionCostSmoothness>();

  // Start pose (10° degrees)
  geometry_msgs::msg::PoseWithCovarianceStamped start_pose =
    geometry_msgs::msg::PoseWithCovarianceStamped();
  start_pose.pose.pose.position.x = 1.0;
  start_pose.pose.pose.position.y = 0.0;
  start_pose.pose.pose.orientation.z = 0.08715574;
  start_pose.pose.pose.orientation.w = 0.9961947;

  // Goal pose (30° degrees)
  geometry_msgs::msg::PoseStamped goal_pose = geometry_msgs::msg::PoseStamped();
  goal_pose.pose.position.x = 2.0;
  goal_pose.pose.position.y = 0.0;
  goal_pose.pose.orientation.z = 0.25881905;
  goal_pose.pose.orientation.w = 0.96592583;

  double expected_smoothness = 20.0 * (M_PI / 180.0);
  double tolerance = 1e-5;

  initialize_move_action_cost(move_action_cost);
  publish_start_pose(start_pose);
  spin_all_nodes();

  // Measure the time nedded to call action cost
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  call_action_cost(goal_pose, msg_test);
  // Measure the time nedded to call action cost
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cerr << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(
    end - begin).count() << "[µs]" << std::endl;

  execute();

  auto current_pose = get_current_pose_in_move_action_cost();

  std::cerr << "Stored pose: " << current_pose.pose.position.x << " " <<
    current_pose.pose.position.y << std::endl;
  std::cerr << "Start pose: " << start_pose.pose.pose.position.x << " " <<
    start_pose.pose.pose.position.y << std::endl;

  ASSERT_DOUBLE_EQ(current_pose.pose.position.x, start_pose.pose.pose.position.x);
  ASSERT_DOUBLE_EQ(current_pose.pose.position.y, start_pose.pose.pose.position.y);
  ASSERT_TRUE(is_message_received());

  auto last_message = get_last_message();
  ASSERT_EQ(last_message->action, "move");

  EXPECT_NEAR((last_message->action_cost).nominal_cost, expected_smoothness, tolerance);
}

TEST_F(TestNode, MoveActionCostTestCostmap)
{
  unsigned char default_costmap_value = 100;
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node_costmap");
  add_node_to_executor(node);
  auto costmap_utils =
    std::make_shared<CostmapUtils>(node, "/global_costmap/costmap", default_costmap_value);

  auto msg_test = std::make_shared<plansys2_msgs::msg::ActionExecution>();
  msg_test->action = "move";
  auto move_action_cost = std::make_shared<plansys2_actions_cost::MoveActionCostMap>();

  // Start pose (10° degrees)
  geometry_msgs::msg::PoseWithCovarianceStamped start_pose =
    geometry_msgs::msg::PoseWithCovarianceStamped();
  start_pose.pose.pose.position.x = 1.0;
  start_pose.pose.pose.position.y = 0.0;
  start_pose.pose.pose.orientation.z = 0.08715574;
  start_pose.pose.pose.orientation.w = 0.9961947;

  // Goal pose (30° degrees)
  geometry_msgs::msg::PoseStamped goal_pose = geometry_msgs::msg::PoseStamped();
  goal_pose.pose.position.x = 2.0;
  goal_pose.pose.position.y = 0.0;
  goal_pose.pose.orientation.z = 0.25881905;
  goal_pose.pose.orientation.w = 0.96592583;

  initialize_move_action_cost(move_action_cost);
  publish_start_pose(start_pose);
  costmap_utils->publish_costmap();
  // rclcpp::spin_some(node->get_node_base_interface());
  spin_all_nodes();

  // Measure the time nedded to call action cost
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  call_action_cost(goal_pose, msg_test);
  // Measure the time nedded to call action cost
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cerr << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(
    end - begin).count() << "[µs]" << std::endl;

  execute();

  auto current_pose = get_current_pose_in_move_action_cost();

  std::cerr << "Stored pose: " << current_pose.pose.position.x << " " <<
    current_pose.pose.position.y << std::endl;
  std::cerr << "Start pose: " << start_pose.pose.pose.position.x << " " <<
    start_pose.pose.pose.position.y << std::endl;

  ASSERT_DOUBLE_EQ(current_pose.pose.position.x, start_pose.pose.pose.position.x);
  ASSERT_DOUBLE_EQ(current_pose.pose.position.y, start_pose.pose.pose.position.y);
  ASSERT_TRUE(is_message_received());

  auto last_message = get_last_message();
  ASSERT_EQ(last_message->action, "move");

  double expected_cost = default_costmap_value * 1 + default_costmap_value * std::pow(1.0, 1);
  double tolerance = 1e-5;
  EXPECT_NEAR((last_message->action_cost).nominal_cost, expected_cost, tolerance);
  /*


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
    */
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
