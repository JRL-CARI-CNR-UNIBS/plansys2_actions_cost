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

namespace plansys2_actions_cost
{

class MoveActionCostLength : public MoveActionCostBase
{
public:
  MoveActionCostLength() {}

  virtual ~MoveActionCostLength() {}

  // void initialize(double param, ) override {};

  void compute_action_cost(const geometry_msgs::msg::PoseStamped & goal) override;

  void update_action_cost() override;

private:
  nav_msgs::msg::Path::SharedPtr path_ptr_;
  PathLength path_length;
  ActionCostPtr compute_cost_function() override;
};

}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_LENGTH_HPP_