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

#ifndef PLANSYS2_ACTIONS_COST__ACTION_COST_BASE_HPP_
#define PLANSYS2_ACTIONS_COST__ACTION_COST_BASE_HPP_

#include <utility>
#include <functional>

#include "plansys2_msgs/msg/action_cost.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

namespace plansys2_actions_cost
{
using ActionCostPtr = plansys2_msgs::msg::ActionCost::SharedPtr;

template<typename T>
class ActionCostBase
{
public:
  ActionCostBase() {}

  virtual ~ActionCostBase() {}

  virtual void initialize(const plansys2::ActionExecutorClient::Ptr & action_executor_client)
  {
    std::cerr << "Initialize action cost base" << std::endl;
    action_executor_client_ = action_executor_client;
  }
  // template<typename... Args>
  // void initialize(Args&&... args)
  // {throw std::runtime_error("The type is not implemented. Aborted");};

  virtual ActionCostPtr compute_action_cost(const T & goal)
  {
    throw std::runtime_error("The type is not implemented. Aborted");
  }

  virtual void update_action_cost() = 0;

protected:
  plansys2::ActionExecutorClient::Ptr action_executor_client_ = nullptr;
  ActionCostPtr action_cost_ptr_ = nullptr;
  
  virtual ActionCostPtr compute_cost_function() = 0;
};

}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__ACTION_COST_BASE_HPP_
