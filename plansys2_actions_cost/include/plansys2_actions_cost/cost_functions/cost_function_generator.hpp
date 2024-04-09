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

#ifndef PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__COST_FUNCTION_GENERATOR_HPP_
#define PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__COST_FUNCTION_GENERATOR_HPP_

#include <utility>
#include <functional>

#include "plansys2_msgs/msg/action_cost.hpp"
// #include "plansys2_executor/ActionExecutorClient.hpp"

namespace plansys2_actions_cost
{
using ActionCostPtr = plansys2_msgs::msg::ActionCost::SharedPtr;
using cost_function_ret_t = ActionCostPtr;
using cost_function_t = std::function<cost_function_ret_t()>;

struct BaseAbstractClass {};

template<typename ... Args>
struct PluginGenerator : BaseAbstractClass
{
  cost_function_ret_t (* f_)(Args ...);

  explicit PluginGenerator(cost_function_ret_t(*f)(Args ...) )
  : f_(f) {}

  cost_function_t args_binder(Args ... args)
  {
    return std::bind(f_, std::forward<Args>(args)...);
  }
};

    #define CUSTOM_PLUGIN_GENERATOR(ObjectName, FunctionPointer, ...) \
  struct ObjectName : PluginGenerator<__VA_ARGS__> \
  { \
    ObjectName() : PluginGenerator(FunctionPointer) {} \
  };

}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__COST_FUNCTIONS__COST_FUNCTION_GENERATOR_HPP_
