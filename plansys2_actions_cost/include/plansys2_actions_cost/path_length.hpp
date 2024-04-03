#ifndef PLANSYS2_ACTIONS_COST__PATH_LENGTH_HPP_
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
