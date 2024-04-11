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

#ifndef PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_BASE_HPP_
#define PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_BASE_HPP_

#include <utility>
#include <functional>

#include "plansys2_msgs/msg/action_cost.hpp"
#include "plansys2_actions_cost/action_cost_base.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace plansys2_actions_cost
{

class MoveActionCostBase : public ActionCostBase<geometry_msgs::msg::PoseStamped>
{
public:
    MoveActionCostBase() {};
    ~MoveActionCostBase() {};

    void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) override;

    virtual void compute_action_cost(const geometry_msgs::msg::PoseStamped & goal, 
                                    const plansys2_msgs::msg::ActionExecution::SharedPtr msg);

    virtual void update_action_cost(){};

protected:
    using ComputePathGoalHandle =
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>;
    using ComputePathFeedback =
        const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Feedback>;
    using ComputePathResult =
        const ComputePathGoalHandle::WrappedResult;

    nav_msgs::msg::Path::SharedPtr path_ptr_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_action_client_;

    virtual ActionCostPtr compute_cost_function() = 0;
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    geometry_msgs::msg::PoseStamped current_pos_;

    void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};
}  // namespace plansys2_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__MOVE_ACTION_COST_BASE_HPP_
