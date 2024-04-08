#ifndef PLANSYS2_ACTIONS_COST__PATH_SMOOTHNESS_HPP_
#define PLANSYS2_ACTIONS_COST__PATH_SMOOTHNESS_HPP_

#include "plansys2_actions_cost/cost_function_generator.hpp"

#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "plansys2_msgs/msg/action_cost.hpp"

namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_smoothness(const nav_msgs::msg::Path::SharedPtr path_ptr);
    CUSTOM_PLUGIN_GENERATOR(PathSmoothness, &compute_path_smoothness, const nav_msgs::msg::Path::SharedPtr)
} // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__PATH_SMOOTHNESS_HPP_

/*
#ifndef PLANSYS2_ACTIONS_COST__PATH_SMOOTHNESS_HPP_
#define PLANSYS2_ACTIONS_COST__PATH_SMOOTHNESS_HPP_

#include "plansys2_actions_cost/base_path_cost_function.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace plansys2_actions_cost
{
    class PathSmoothness : public BasePathCostFunction
    {
    public:
        explicit PathSmoothness(const nav_msgs::msg::Path::SharedPtr& path)            
            : BasePathCostFunction(path) {} 

        plansys2_msgs::msg::ActionCost cost_function() override;

    };

} // namespace plansys_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__PATH_SMOOTHNESS_HPP_

*/