#include "plansys2_actions_cost/cumulative_path_distance.hpp"

namespace plansys2_actions_cost
{
    CumulativePathDistance::CumulativePathDistance()
    {
    }

    plansys2_msgs::msg::ActionCost CumulativePathDistance::cost_function(const nav2_msgs::msg::Path& path)
    {
        plansys2_msgs::msg::ActionCost action_cost;

        action_cost.nominal_cost = nav2_util::geometry_utils::calculate_path_length(path);
        action_cost.std_dev = 0.0;
        
        return action_cost;    
    }

} // namespace plansys2_actions_cost