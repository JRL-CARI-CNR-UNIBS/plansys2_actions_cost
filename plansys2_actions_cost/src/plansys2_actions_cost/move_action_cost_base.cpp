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

#include "plansys2_actions_cost/move_action_cost_base.hpp"

namespace plansys2_actions_cost
{

void MoveActionCostBase::initialize(
  const plansys2::ActionExecutorClient::Ptr & action_executor_client)
{
  current_pose_.pose.position.x = 10.0;
  if (action_executor_client == nullptr) {
    std::cerr << "Action executor client is nullptr" << std::endl;
    return;
  }
  action_executor_client_ = action_executor_client;

  // Retrieve namespace parameter
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.name = "namespace";
  param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  param_desc.description =
    "Namespace of the action cost plugin. Default: ''";

  action_executor_client_->declare_parameter("namespace", "", param_desc);
  action_executor_client_->get_parameter("namespace", namespace_);

  compute_path_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
    action_executor_client_->shared_from_this(),
    namespace_ + "/compute_path_to_pose");

  RCLCPP_INFO(action_executor_client_->get_logger(), "Waiting for compute path action server...");

  bool is_action_server_ready =
    compute_path_action_client_->wait_for_action_server(std::chrono::seconds(5));

  if (is_action_server_ready) {
    RCLCPP_INFO(action_executor_client_->get_logger(), "Compute path action server ready");
  } else {
    RCLCPP_ERROR(
      action_executor_client_->get_logger(),
      "Compute path action server not available after waiting");
  }

  path_pub_ = action_executor_client_->create_publisher<nav_msgs::msg::Path>(
    namespace_ + "/computed_path", 10);
  pose_sub_ =
    action_executor_client_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    namespace_ + "/amcl_pose",
    10,
    std::bind(&MoveActionCostBase::current_pose_callback, this, std::placeholders::_1));
  RCLCPP_DEBUG(action_executor_client_->get_logger(), "[MoveActionCostBase] Correctly initialized");
}

void MoveActionCostBase::compute_action_cost(
  const geometry_msgs::msg::PoseStamped & goal,
  const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();

  // Feedback callbkack
  send_goal_options.feedback_callback = [this](
    ComputePathGoalHandle::SharedPtr goal_handle,
    ComputePathFeedback feedback)
    {
      RCLCPP_DEBUG(
        action_executor_client_->get_logger(),
        "Feedback received from action cost computation (computing)");
    };
  // Result callbkack -> Path
  send_goal_options.result_callback = [this, msg](ComputePathResult & result)
    {
      ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();

      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_DEBUG(
          action_executor_client_->get_logger(),
          "Compute path to cost. Goal failed with error code %d",
          static_cast<int>(result.code));

        action_cost->nominal_cost = std::numeric_limits<double>::infinity();
        action_cost->std_dev_cost = 0.0;

        this->action_executor_client_->set_action_cost(action_cost, msg);
        return;
      }

      auto path = result.result->path;
      path_ptr_ = std::make_shared<nav_msgs::msg::Path>(path);
      action_cost = this->compute_cost_function();
      this->action_executor_client_->set_action_cost(action_cost, msg);

      RCLCPP_DEBUG(
        action_executor_client_->get_logger(), "Computed path cost: %f", action_cost->nominal_cost);
      path_pub_->publish(path);
    };

  send_goal_options.goal_response_callback =
    [this, msg](const ComputePathGoalHandle::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_DEBUG(action_executor_client_->get_logger(), "Goal was rejected by server");

        ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
        action_cost->nominal_cost = std::numeric_limits<double>::infinity();
        action_cost->std_dev_cost = 0.0;
        this->action_executor_client_->set_action_cost(action_cost, msg);
      } else {
        RCLCPP_DEBUG(
          action_executor_client_->get_logger(), "Goal accepted by server, waiting for result");
      }
    };
  nav2_msgs::action::ComputePathToPose::Goal path_to_pose_goal =
    nav2_msgs::action::ComputePathToPose::Goal();
  // TODO(samuele): Retrieve this as a parameter in initialize method
  path_to_pose_goal.planner_id = "GridBased";
  path_to_pose_goal.start.header.frame_id = "map";
  path_to_pose_goal.start = current_pose_;
  path_to_pose_goal.goal = goal;

  auto future_goal_handle = compute_path_action_client_->async_send_goal(
    path_to_pose_goal,
    send_goal_options);

  RCLCPP_DEBUG(
    action_executor_client_->get_logger(),
    "Start pose: %f %f", path_to_pose_goal.start.pose.position.x,
    path_to_pose_goal.start.pose.position.y);
  RCLCPP_DEBUG(
    action_executor_client_->get_logger(),
    "Goal pose: %f %f", path_to_pose_goal.goal.pose.position.x,
    path_to_pose_goal.goal.pose.position.y);
  RCLCPP_DEBUG(action_executor_client_->get_logger(), "Waiting for result...");
}

void MoveActionCostBase::current_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_DEBUG(
    action_executor_client_->get_logger(),
    "[MoveActionCostBase] Callback received");

  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;

  std::cerr << "Current pose: " << current_pose_.pose.position.x << " " <<
    current_pose_.pose.position.y << std::endl;
  RCLCPP_DEBUG(
    action_executor_client_->get_logger(),
    "[MoveActionCostBase] Current pose: %f %f", current_pose_.pose.position.x,
    current_pose_.pose.position.y);
}

}  // namespace plansys2_actions_cost
