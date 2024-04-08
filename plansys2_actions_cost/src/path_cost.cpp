#include "plansys2_actions_cost/path_cost.hpp"

namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_cost(const nav_msgs::msg::Path::SharedPtr& path_ptr, 
                                    const std::shared_ptr<nav2_costmap_2d::Costmap2D>& costmap,
                                    const double& lambda)
    {
        ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
        if(costmap == nullptr)
        {
            // RCLCPP_ERROR(get_logger(), "Costmap subscriber is null");
            return action_cost;
        }

        unsigned int mx = 0;
        unsigned int my = 0;
        unsigned int cost = 0.0;
        double cumulative_cost = 0.0;
        unsigned int n_pose = 0;
        for (const auto& pose : path_ptr->poses)
        {
            costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
            cost = costmap->getCost(mx, my);
            
            if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                action_cost->nominal_cost = std::numeric_limits<double>::infinity();
                return action_cost;
            }
            double weight = std::pow(lambda, n_pose);

            cumulative_cost += static_cast<double>(cost) * weight;
            n_pose++;
        }
        action_cost->nominal_cost = cumulative_cost;
        return action_cost;
    }
}   // namespace plansys2_actions_cost


// namespace plansys2_actions_cost
// {

//     plansys2_msgs::msg::ActionCost PathLength::cost_function()
//     {
//         plansys2_msgs::msg::ActionCost action_cost;

//         if (path_->poses.size() < 2)
//         {
//             return action_cost;
//         }

//         action_cost.nominal_cost = nav2_util::geometry_utils::calculate_path_length(*path_);
//         action_cost.std_dev_cost = 0.0;

//         return action_cost;
//     }

// } // namespace plansys2_actions_cost
