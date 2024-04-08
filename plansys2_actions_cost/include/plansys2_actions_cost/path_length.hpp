#ifndef PLANSYS2_ACTIONS_COST__PATH_LENGTH_HPP_
#define PLANSYS2_ACTIONS_COST__PATH_LENGTH_HPP_

#include "plansys2_actions_cost/cost_function_generator.hpp"

#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "plansys2_msgs/msg/action_cost.hpp"

namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_length(const nav_msgs::msg::Path::SharedPtr& path_ptr);
    CUSTOM_PLUGIN_GENERATOR(PathLength, &compute_path_length, const nav_msgs::msg::Path::SharedPtr&)
} // namespace plansys2_actions_cost


#endif  // PLANSYS2_ACTIONS_COST__PATH_LENGTH_HPP_




/*#ifndef PLANSYS2_ACTIONS_COST__PATH_LENGTH_HPP_
#define PLANSYS2_ACTIONS_COST__PATH_LENGTH_HPP_

#include "plansys2_actions_cost/base_path_cost_function.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace plansys2_actions_cost
{
    class PathLength : public BasePathCostFunction
    {
    public:
        explicit PathLength(const nav_msgs::msg::Path::SharedPtr& path)
            : BasePathCostFunction(path) {} 
            
        plansys2_msgs::msg::ActionCost cost_function() override;
    };

} // namespace plansys_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__PATH_LENGTH_HPP_
*/