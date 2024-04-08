#ifndef PLANSYS2_ACTIONS_COST__PATH_COST_HPP_
#define PLANSYS2_ACTIONS_COST__PATH_COST_HPP_

#include "plansys2_actions_cost/cost_function_generator.hpp"

#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "plansys2_msgs/msg/action_cost.hpp"

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_cost(const nav_msgs::msg::Path::SharedPtr& path_ptr, 
                                    const std::shared_ptr<nav2_costmap_2d::Costmap2D>& costmap,
                                    const double& lambda);
    CUSTOM_PLUGIN_GENERATOR(PathCost, &compute_path_cost, const nav_msgs::msg::Path::SharedPtr&, 
                                                          const std::shared_ptr<nav2_costmap_2d::Costmap2D>&, 
                                                          const double&)
} // namespace plansys2_actions_cost


/* Alternative solution
namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_cost(const nav_msgs::msg::Path::SharedPtr& path_ptr,
                                    const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>& costmap_subscriber,
                                    const double& lambda);
} // namespace plansys2_actions_cost

namespace plansys2_actions_cost
{
    ActionCostPtr compute_path_cost(const nav_msgs::msg::Path::SharedPtr& path_ptr,
                                    const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>& costmap_subscriber,
                                    const double& lambda)
    {
        using Clock = std::chrono::high_resolution_clock;
        auto start_time = Clock::now();

        auto costmap_subscriber = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
        node->shared_from_this(), 
        "/fake_cost_map_raw");

        auto end_time = Clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        std::cout << "Tempo impiegato per la sottoscrizione al costmap: " << duration.count() << " millisecondi" << std::endl;

        ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
        return action_cost;
    }
    CUSTOM_PLUGIN_GENERATOR(PathCost, &compute_path_cost, const nav_msgs::msg::Path::SharedPtr&, const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>&, const double&)
}   // namespace plansys2_actions_cost
*/
#endif  // PLANSYS2_ACTIONS_COST__PATH_COST_HPP_


// namespace plansys2_actions_cost
// {
//     ActionCostPtr compute_path_cost(const nav_msgs::msg::Path::SharedPtr path_ptr, 
//                                     const std::unique_ptr<nav2_costmap_2d::CostmapSubscriber>& costmap_subscriber,
//                                     const double& lambda);
// } // namespace plansys2_actions_cost

// namespace plansys2_actions_cost
// {
//     ActionCostPtr compute_path_cost(const nav_msgs::msg::Path::SharedPtr path_ptr, 
//                                     const std::unique_ptr<nav2_costmap_2d::CostmapSubscriber>& costmap_subscriber,
//                                     const double& lambda)
//     {
//         ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
//         if(costmap_subscriber == nullptr)
//         {
//             // RCLCPP_ERROR(get_logger(), "Costmap subscriber is null");
//             return action_cost;
//         }

//         bool costmap_ready = false;
//         std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = nullptr;

//         while (!costmap_ready && rclcpp::ok()) {
//             try {
//                 costmap = costmap_subscriber->getCostmap();
//                 if (costmap) {
//                     costmap_ready = true;
//                 } else {
//                     action_cost->nominal_cost = std::numeric_limits<double>::infinity();
//                     return action_cost;
//                 }
//             } catch (const std::exception& e) {
//                 // RCLCPP_INFO(get_logger(), "Waiting costmap");
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//             }
//         }

//         unsigned int mx = 0;
//         unsigned int my = 0;
//         unsigned int cost = 0.0;
//         double cumulative_cost = 0.0;
//         unsigned int n_pose = 0;
//         for (const auto& pose : path_ptr->poses)
//         {
//             costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
//             cost = costmap->getCost(mx, my);
            
//             if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
//             {
//                 action_cost->nominal_cost = std::numeric_limits<double>::infinity();
//                 return action_cost;
//             }
//             double weight = std::pow(lambda, n_pose);
//             // costs.push_back(static_cast<double>(cost));
//             cumulative_cost += static_cast<double>(cost) * weight;
//         }
//         action_cost->nominal_cost = cumulative_cost;
//         return action_cost;
//     }
//     CUSTOM_PLUGIN_GENERATOR(PathCost, &compute_path_cost, const nav_msgs::msg::Path::SharedPtr, 
//                                                           const std::unique_ptr<nav2_costmap_2d::CostmapSubscriber>&, 
//                                                           const double&)
// }   // namespace plansys2_actions_cost
// #endif  // PLANSYS2_ACTIONS_COST__PATH_COST_HPP_

