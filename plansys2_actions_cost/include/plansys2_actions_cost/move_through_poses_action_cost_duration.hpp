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

#ifndef PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_DURATION_HPP_
#define PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_DURATION_HPP_

#include <utility>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "plansys2_msgs/msg/action_cost.hpp"
#include "plansys2_actions_cost/move_through_poses_action_cost_base.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "nav2_util/geometry_utils.hpp"

namespace plansys2_actions_cost
{

class MoveThroughPosesActionCostDuration : public MoveThroughPosesActionCostBase
{

public:
  MoveThroughPosesActionCostDuration(){}
  ~MoveThroughPosesActionCostDuration() {}
  void initialize(const plansys2::ActionExecutorClient::Ptr & action_executor_client) override;

protected:
  inline ActionCostPtr compute_cost_function() override;

private:
  double vx_max_, vy_max_, wz_max_;
  bool use_rotations_;
};

}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_DURATION_HPP_

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  plansys2_actions_cost::MoveThroughPosesActionCostDuration,
  plansys2_actions_cost::MoveThroughPosesActionCostBase)
