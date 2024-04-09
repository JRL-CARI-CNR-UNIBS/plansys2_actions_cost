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

#ifndef PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__PATH_LENGTH_HPP_
#define PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__PATH_LENGTH_HPP_

#include "plansys2_actions_cost/cost_functions/cost_function_generator.hpp"

#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "plansys2_msgs/msg/action_cost.hpp"

namespace plansys2_actions_cost
{
ActionCostPtr compute_path_length(const nav_msgs::msg::Path::SharedPtr & path_ptr);
CUSTOM_PLUGIN_GENERATOR(PathLength, &compute_path_length, const nav_msgs::msg::Path::SharedPtr &)
}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__PATH_LENGTH_HPP_
