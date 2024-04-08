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

#include "plansys2_actions_cost/path_smoothness.hpp"

namespace plansys2_actions_cost
{
ActionCostPtr compute_path_smoothness(const nav_msgs::msg::Path::SharedPtr & path_ptr)
{
  ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
  if (path_ptr->poses.size() < 2) {
    return action_cost;
  }
  action_cost->nominal_cost = 0.0;

  tf2::Quaternion quat_k, quat_km1, relative_quat;
  for (size_t k = 1; k < path_ptr->poses.size(); k++) {
    tf2::fromMsg(path_ptr->poses[k].pose.orientation, quat_k);
    tf2::fromMsg(path_ptr->poses[k - 1].pose.orientation, quat_km1);

    relative_quat = quat_km1.inverse() * quat_k;

    auto rotation_angle = relative_quat.getAngleShortestPath();
    auto rotation_axis = relative_quat.getAxis();
    auto relative_angle = (rotation_angle * rotation_axis).length();  // norm of the vector

    action_cost->nominal_cost += fabs(relative_angle);
  }
  return action_cost;
}
}  // namespace plansys2_actions_cost
