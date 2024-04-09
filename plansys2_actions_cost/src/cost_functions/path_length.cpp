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

#include "plansys2_actions_cost/cost_functions/path_length.hpp"

namespace plansys2_actions_cost
{
ActionCostPtr compute_path_length(const nav_msgs::msg::Path::SharedPtr & path_ptr)
{
  ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();

  action_cost->nominal_cost = nav2_util::geometry_utils::calculate_path_length(*path_ptr);
  action_cost->std_dev_cost = 0.0;

  return action_cost;
}
}  // namespace plansys2_actions_cost
