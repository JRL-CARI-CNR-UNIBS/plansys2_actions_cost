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

#ifndef PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_MAP_HPP_
#define PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_MAP_HPP_

#include <utility>
#include <functional>
#include <memory>

#include "plansys2_msgs/msg/action_cost.hpp"
#include "plansys2_actions_cost/cost_functions/path_cost.hpp"
#include "plansys2_actions_cost/move_action_cost_base.hpp"

namespace plansys2_actions_cost
{

class MoveActionCostMap : public MoveActionCostBase
{
public:
  MoveActionCostMap()
  : lambda_(1.0) {}
  ~MoveActionCostMap() {}

  void initialize(const plansys2::ActionExecutorClient::Ptr & action_executor_client) override;

protected:
  PathCost path_cost;
  double lambda_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;

  ActionCostPtr compute_cost_function() override;
  /*
  {
    if (!costmap_) {
      std::cerr << "MoveActionCostMap: costmap not set" << std::endl;
      return nullptr;
    }
    auto cost_function = path_cost.args_binder(std::ref(path_ptr_),
                                               std::ref(costmap_),
                                               std::ref(lambda_));
    return cost_function();
  } */
};

}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_MAP_HPP_

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  plansys2_actions_cost::MoveActionCostMap,
  plansys2_actions_cost::MoveActionCostBase)
