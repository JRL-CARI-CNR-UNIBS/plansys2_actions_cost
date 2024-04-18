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

#ifndef PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_LENGTH_HPP_
#define PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_LENGTH_HPP_

#include <utility>
#include <functional>

#include "plansys2_msgs/msg/action_cost.hpp"
#include "plansys2_actions_cost/cost_functions/path_length.hpp"
#include "plansys2_actions_cost/move_action_cost_base.hpp"

namespace plansys2_actions_cost
{

class MoveActionCostLength : public MoveActionCostBase
{
public:
  MoveActionCostLength() {}
  ~MoveActionCostLength() {}

protected:
  PathLength path_length;
  inline ActionCostPtr compute_cost_function() override
  {
    auto cost_function = path_length.args_binder(std::ref(path_ptr_));
    return cost_function();
  }
};

}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_LENGTH_HPP_

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  plansys2_actions_cost::MoveActionCostLength,
  plansys2_actions_cost::MoveActionCostBase)
