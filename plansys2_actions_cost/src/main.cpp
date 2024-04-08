#include "plansys2_actions_cost/path_length.hpp"
#include "plansys2_actions_cost/path_smoothness.hpp"
#include "plansys2_actions_cost/path_cost.hpp"

#include "plansys2_msgs/msg/action_cost.hpp"
#include "nav_msgs/msg/path.hpp"
#include <iostream>
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

plansys2_msgs::msg::ActionCost::SharedPtr generic_function_that_uses_the_cost_function(plansys2_actions_cost::cost_function_t cost_function)
{
    std::cout << "OK" << std::endl;
    return cost_function();
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // nav_msgs::msg::Path::SharedPtr test_path_ptr(new nav_msgs::msg::Path);
    // test_path_ptr->poses.resize(2);

    // test_path_ptr->poses[0].pose.position.x = 0.0;
    // test_path_ptr->poses[0].pose.position.y = 0.0;
    // test_path_ptr->poses[1].pose.position.x = 1.0;
    // test_path_ptr->poses[1].pose.position.y = 1.0;

    // plansys2_actions_cost::PathLength g;
    
    // auto h = g.args_binder(std::ref(test_path_ptr));
    // std::cout << generic_function_that_uses_the_cost_function(h)->nominal_cost << std::endl;


    nav_msgs::msg::Path::SharedPtr test_path_ptr(new nav_msgs::msg::Path);
    test_path_ptr->poses.resize(2);

    
    // Pose 1: senza rotazione
    // test_path_ptr->poses[0].pose.position.x = 0.0;
    // test_path_ptr->poses[0].pose.position.y = 0.0;

    // Pose 2: ruotata di 10° attorno all'asse z
    test_path_ptr->poses[0].pose.position.x = 1.0;
    test_path_ptr->poses[0].pose.position.y = 0.0;
    test_path_ptr->poses[0].pose.orientation.z = 0.08715574;  
    test_path_ptr->poses[0].pose.orientation.w = 0.9961947;  

    // test_path_ptr->poses[1].pose.orientation.z = 0.173648;  
    // test_path_ptr->poses[1].pose.orientation.w = 0.984808;  

    // Pose 3: ruotata di 30° attorno all'asse z
    test_path_ptr->poses[1].pose.position.x = 2.0;
    test_path_ptr->poses[1].pose.position.y = 0.0;
    test_path_ptr->poses[1].pose.orientation.z = 0.25881905;  
    test_path_ptr->poses[1].pose.orientation.w = 0.96592583;  

    // test_path_ptr->poses[2].pose.orientation.z = 0.258819;  
    // test_path_ptr->poses[2].pose.orientation.w = 0.965926; 

    // Calcolo della smoothness del percorso
    plansys2_actions_cost::PathSmoothness g;
    auto h = g.args_binder(std::ref(test_path_ptr));
    std::cout << generic_function_that_uses_the_cost_function(h)->nominal_cost << std::endl;

    auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");

    // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    auto costmap_subscriber = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
      node->shared_from_this(), 
      "/fake_cost_map_raw");
    unsigned char default_value = 100; 
    auto costmap_to_send = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
    // costmap_to_send->resetMaps();
    // auto costmap_to_send = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
    for (unsigned int y = 0; y < costmap_to_send->getSizeInCellsY(); y++) {
        for (unsigned int x = 0; x < costmap_to_send->getSizeInCellsX(); x++) {
            costmap_to_send->setCost(x, y, default_value);
        }
    }
    // auto costmapPublisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    //     node, costmapToSend, "", "/costmap_test", true);

    auto costmapPublisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
        node->shared_from_this(), costmap_to_send.get(), "", "/fake_cost_map", true);

    costmapPublisher->on_activate();
    
    // costmapPublisher->setCostmap(costmapToSend);
    auto rate = rclcpp::Rate(1);
    while (rclcpp::ok()) {
        // costmapPublisher->setCostmap(costmapToSend);
        // costmapPublisher->createCostmapUpdateMsg();
        costmapPublisher->publishCostmap();
        rclcpp::spin_some(node->get_node_base_interface());
        RCLCPP_INFO(node->get_logger(), "Waiting costmap");
        auto costmap = costmap_subscriber->getCostmap();
        if(costmap)
        {
            std::cout << costmap->getSizeInCellsX() << std::endl;
            RCLCPP_INFO(node->get_logger(), "Costmap available");
            double lamb = 0.5;
            std::cout << lamb << std::endl;
            plansys2_actions_cost::PathCost path_cost;
            auto path_cost_function = path_cost.args_binder(std::ref(test_path_ptr), std::ref(costmap), std::ref(lamb));

            double action_cost;
            action_cost = path_cost_function()->nominal_cost;
            std::cout << action_cost << std::endl;

        }
        else
        {
            // std::cout << "Costmap not available" << std::endl;
            RCLCPP_INFO(node->get_logger(), "Costmap NOT available");

        }
        rate.sleep();
    }
    
    // rclcpp::spin_some(node->get_node_base_interface());
    // // std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = costmap_subscriber->getCostmap();
    // std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = nullptr;
    // bool costmap_ready = false;
    // while (!costmap_ready && rclcpp::ok()) {
    //     try {
    //         costmap = costmap_subscriber->getCostmap();
    //         if (costmap) {
    //             costmap_ready = true;
    //         } else {
                
    //             // action_cost->nominal_cost = std::numeric_limits<double>::infinity();
    //             // return action_cost;
    //         }
    //     } catch (const std::exception& e) {
    //         rclcpp::spin_some(node->get_node_base_interface());
    //         // RCLCPP_INFO(get_logger(), "Waiting costmap");
    //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     }
    // }


    // nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);
    // path_ptr->poses.resize(3);

    // // Define sample poses
    // geometry_msgs::msg::PoseStamped pose1, pose2, pose3;
    // pose1.pose.position.x = 0.0;
    // pose1.pose.position.y = 0.0;
    // pose2.pose.position.x = 1.0;
    // pose2.pose.position.y = 1.0;
    // pose3.pose.position.x = 2.0;
    // pose3.pose.position.y = 2.0;

    // // Add poses to the path
    // path_ptr->poses[0] = pose1;
    // path_ptr->poses[1] = pose2;
    // path_ptr->poses[2] = pose3;
    // double lambda = 1; // Sample lambda value
    // plansys2_actions_cost::PathCost path_cost;
    // double lambda2 = 0.1;
    // auto path_cost_function = path_cost.args_binder(std::ref(path_ptr), std::ref(costmap),std::ref(lambda));

    // double action_cost;
    // action_cost = path_cost_function()->nominal_cost;
    // std::cout << action_cost << std::endl;

    // Create an instance of the mock costmap subscriber
    // auto mock_costmap_subscriber = std::make_unique<MockCostmapSubscriber>();

    // Call the function being tested
    // double lambda = 0.5; // Sample lambda value
    // plansys2_actions_cost::PathCost path_cost;
    // double lambda2 = 0.1;
    // auto path_cost_function = path_cost.args_binder(std::ref(path_ptr), std::ref(costmap_subscriber),std::ref(lambda));

    // double action_cost = path_cost_function()->nominal_cost;
    // std::cout << action_cost << std::endl;


    return 0;
}

