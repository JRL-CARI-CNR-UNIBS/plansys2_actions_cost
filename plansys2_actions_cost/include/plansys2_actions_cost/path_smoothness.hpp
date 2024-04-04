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
} // namespace plansys2_actions_cost


namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_smoothness(const nav_msgs::msg::Path::SharedPtr path_ptr)
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
    CUSTOM_PLUGIN_GENERATOR(PathSmoothness, &compute_path_smoothness, const nav_msgs::msg::Path::SharedPtr)
}   // namespace plansys2_actions_cost
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