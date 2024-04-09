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

void MoveActionCostLength::compute_action_cost(const geometry_msgs::msg::PoseStamped & goal)
{
  // TODO(samuele): Action Server call
}

ActionCostPtr MoveActionCostLength::compute_cost_function()
{
  auto cost_function = path_length.args_binder(std::ref(path_ptr_));
  return cost_function();
}

}  // namespace plansys2_actions_cost
