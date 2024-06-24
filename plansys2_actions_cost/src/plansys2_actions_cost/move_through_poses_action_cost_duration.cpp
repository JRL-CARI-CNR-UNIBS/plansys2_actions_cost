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

#include "plansys2_actions_cost/move_through_poses_action_cost_duration.hpp"

namespace plansys2_actions_cost
{

void MoveThroughPosesActionCostDuration::initialize(
  const plansys2::ActionExecutorClient::Ptr & action_executor_client)
{
  if (action_executor_client == nullptr) {
    std::cerr << "Action executor client is nullptr" << std::endl;
    return;
  }
  MoveThroughPosesActionCostBase::initialize(action_executor_client);

  auto declare_parameter = [&](const std::string& param_name, auto default_val, auto type,
                                  const std::string& description) -> auto
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.set__description(description);
    desc.type = type;
    return action_executor_client->declare_parameter(param_name, default_val, desc);
  };

  declare_parameter("vx_max", 0.5, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, "Robot velocity x-direction");
  declare_parameter("vy_max", 0.0, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, "Robot velocity y-direction");
  declare_parameter("wz_max", 2.0, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, "Robot velocity wz-direction");
  declare_parameter("use_rotations", false, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, "Use rotations in duration computation");

  action_executor_client->get_parameter("vx_max", vx_max_);
  action_executor_client->get_parameter("vy_max", vy_max_);
  action_executor_client->get_parameter("wz_max", wz_max_);
  action_executor_client->get_parameter("use_rotations", use_rotations_);
}

ActionCostPtr MoveThroughPosesActionCostDuration::compute_cost_function()
{
  RCLCPP_INFO(action_executor_client_->get_logger(), "Compute cost function duration");
  ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
  if (path_ptr_->poses.size() < 2) {
    return action_cost;
  }
  
  double path_translation_length = 0.0;
  double path_rotation_length = 0.0;
  tf2::Quaternion quat_k, quat_km1, relative_quat;

  for (size_t k = 1; k < path_ptr_->poses.size(); k++) {
    tf2::fromMsg(path_ptr_->poses[k].pose.orientation, quat_k);
    tf2::fromMsg(path_ptr_->poses[k - 1].pose.orientation, quat_km1);

    relative_quat = quat_km1.inverse() * quat_k;

    auto rotation_angle = relative_quat.getAngleShortestPath();
    auto rotation_axis = relative_quat.getAxis();
    auto relative_angle = (rotation_angle * rotation_axis).length();  // norm of the vector

    path_rotation_length += fabs(relative_angle);
    path_translation_length += nav2_util::geometry_utils::euclidean_distance(path_ptr_->poses[k].pose, path_ptr_->poses[k - 1].pose);
  }
  if(path_translation_length < nav2_util::geometry_utils::calculate_path_length(*path_ptr_) - 1e-5 || path_translation_length > nav2_util::geometry_utils::calculate_path_length(*path_ptr_) + 1e-5 ){
    throw std::runtime_error("Path translation length is not equal to the path length");
  }
  
  action_cost->nominal_cost = path_translation_length / std::hypot(vx_max_, vy_max_) + path_rotation_length / wz_max_; 
  std::cerr << "PATH DURATION: " << action_cost->nominal_cost << std::endl;
  return action_cost;
}


}  // namespace plansys2_actions_cost