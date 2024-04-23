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

#include "plansys2_actions_cost/move_action_cost_map.hpp"

namespace plansys2_actions_cost
{

void MoveActionCostMap::initialize(
  const plansys2::ActionExecutorClient::Ptr & action_executor_client)
{
  if (action_executor_client == nullptr) {
    std::cerr << "Action executor client is nullptr" << std::endl;
    return;
  }
  MoveActionCostBase::initialize(action_executor_client);

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.name = "lambda";
  param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  param_desc.description =
    "Lambda parameter for weigthed sum of costs ∑_i(c_i * λ^i). Must be 0<lambda<=1. Default: 1.0";

  action_executor_client_->declare_parameter("lambda", 1.0, param_desc);
  action_executor_client_->get_parameter("lambda", lambda_);

  if (lambda_ < 0.0 || lambda_ > 1.0) {
    RCLCPP_WARN(
      action_executor_client_->get_logger(), "Lambda value must be: 0<lambda<=1. Setting to 1.0");
    lambda_ = 1.0;
  }
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    action_executor_client_->shared_from_this(),
    namespace_ + "/global_costmap/costmap_raw");  // fake_cost_map_raw


  RCLCPP_DEBUG(action_executor_client_->get_logger(), "[MoveActionCostBase] Correctly initialized");
}

ActionCostPtr MoveActionCostMap::compute_cost_function()
{
  try {
    costmap_ = costmap_sub_->getCostmap();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(action_executor_client_->get_logger(), "Error getting costmap: %s", e.what());
    std::cerr << "Error getting costmap: " << e.what() << std::endl;
    return nullptr;
  }
  if (!costmap_) {
    std::cerr << "MoveActionCostMap: costmap not set" << std::endl;
    return nullptr;
  }
  auto cost_function = path_cost.args_binder(
    std::ref(path_ptr_),
    std::ref(costmap_),
    std::ref(lambda_));
  return cost_function();
}

}  // namespace plansys2_actions_cost
