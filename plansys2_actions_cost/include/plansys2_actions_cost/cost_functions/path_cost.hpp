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

#ifndef PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__PATH_COST_HPP_
#define PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__PATH_COST_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <memory>

#include "plansys2_actions_cost/cost_functions/cost_function_generator.hpp"

#include "plansys2_msgs/msg/action_cost.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace plansys2_actions_cost
{
ActionCostPtr compute_path_cost(
  const nav_msgs::msg::Path::SharedPtr & path_ptr,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap,
  const double & lambda);
CUSTOM_PLUGIN_GENERATOR(
  PathCost, &compute_path_cost, const nav_msgs::msg::Path::SharedPtr &,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> &,
  const double &)
}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__PATH_COST_HPP_
