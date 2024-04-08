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

#include <iostream>

#include "plansys2_actions_cost/path_length.hpp"
#include "plansys2_actions_cost/path_smoothness.hpp"
#include "plansys2_actions_cost/path_cost.hpp"


#include "plansys2_msgs/msg/action_cost.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

plansys2_msgs::msg::ActionCost::SharedPtr generic_function_that_uses_the_cost_function(
  plansys2_actions_cost::cost_function_t cost_function)
{
  std::cout << "OK" << std::endl;
  return cost_function();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  nav_msgs::msg::Path::SharedPtr test_path_ptr(new nav_msgs::msg::Path);
  test_path_ptr->poses.resize(2);


  test_path_ptr->poses[0].pose.position.x = 1.0;
  test_path_ptr->poses[0].pose.position.y = 0.0;
  test_path_ptr->poses[0].pose.orientation.z = 0.08715574;
  test_path_ptr->poses[0].pose.orientation.w = 0.9961947;

  test_path_ptr->poses[1].pose.position.x = 2.0;
  test_path_ptr->poses[1].pose.position.y = 0.0;
  test_path_ptr->poses[1].pose.orientation.z = 0.25881905;
  test_path_ptr->poses[1].pose.orientation.w = 0.96592583;

  plansys2_actions_cost::PathSmoothness g;
  auto h = g.args_binder(std::ref(test_path_ptr));
  std::cout << generic_function_that_uses_the_cost_function(h)->nominal_cost << std::endl;

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");

  auto costmap_subscriber = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    node->shared_from_this(),
    "/fake_cost_map_raw");
  unsigned char default_value = 100;
  auto costmap_to_send = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);

  for (unsigned int y = 0; y < costmap_to_send->getSizeInCellsY(); y++) {
    for (unsigned int x = 0; x < costmap_to_send->getSizeInCellsX(); x++) {
      costmap_to_send->setCost(x, y, default_value);
    }
  }

  auto costmapPublisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node->shared_from_this(), costmap_to_send.get(), "", "/fake_cost_map", true);

  costmapPublisher->on_activate();

  auto rate = rclcpp::Rate(1);
  while (rclcpp::ok()) {
    costmapPublisher->publishCostmap();
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(node->get_logger(), "Waiting costmap");
    auto costmap = costmap_subscriber->getCostmap();
    if (costmap) {
      std::cout << costmap->getSizeInCellsX() << std::endl;
      RCLCPP_INFO(node->get_logger(), "Costmap available");
      double lamb = 0.5;
      std::cout << lamb << std::endl;
      plansys2_actions_cost::PathCost path_cost;
      auto path_cost_function = path_cost.args_binder(
        std::ref(test_path_ptr), std::ref(
          costmap), std::ref(lamb));

      double action_cost;
      action_cost = path_cost_function()->nominal_cost;
      std::cout << action_cost << std::endl;
    } else {
      RCLCPP_INFO(node->get_logger(), "Costmap NOT available");
    }
    rate.sleep();
  }
  return 0;
}
