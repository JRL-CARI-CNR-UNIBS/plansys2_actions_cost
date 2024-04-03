#ifndef PLANSYS2_ACTIONS_COST__PATH_LENGTH2_HPP_
#define PLANSYS2_ACTIONS_COST__PATH_LENGTH2_HPP_

#include "plansys2_actions_cost/cost_function_generator.hpp"

#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "plansys2_msgs/msg/action_cost.hpp"

namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_test(const nav_msgs::msg::Path::SharedPtr path_ptr)
    {
        ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();

        action_cost->nominal_cost = nav2_util::geometry_utils::calculate_path_length(*path_ptr);
        action_cost->std_dev_cost = 0.0;

        return action_cost;
    }
    CUSTOM_PLUGIN_GENERATOR(PathLength2, &compute_path_test, const nav_msgs::msg::Path::SharedPtr)

} // namespace plansys_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__PATH_LENGTH2_HPP_


