#include "plansys2_actions_cost/path_smoothness.hpp"

namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_smoothness(const nav_msgs::msg::Path::SharedPtr& path_ptr)
    {
        ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
        if (path_ptr->poses.size() < 2)
        {
            return action_cost;
        }
        action_cost->nominal_cost = 0.0;

        tf2::Quaternion quat_k, quat_km1, relative_quat;
        for(size_t k = 1; k < path_ptr->poses.size(); k++)
        {
            tf2::fromMsg(path_ptr->poses[k].pose.orientation, quat_k);
            tf2::fromMsg(path_ptr->poses[k-1].pose.orientation, quat_km1);
            
            relative_quat = quat_km1.inverse() * quat_k;

            auto rotation_angle = relative_quat.getAngleShortestPath();
            auto rotation_axis = relative_quat.getAxis();
            auto relative_angle = (rotation_angle*rotation_axis).length();  //norm of the vector
            // getAngleShortestPath(relative_quat);
            // if(relative_quat.getW() < 0)
            // {
            //     relative_quat = -relative_quat;
            // }

            action_cost->nominal_cost += fabs(relative_angle);
        }
        return action_cost;
    }
}   // namespace plansys2_actions_cost

// namespace plansys2_actions_cost
// {

//     plansys2_msgs::msg::ActionCost PathSmoothness::cost_function()
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