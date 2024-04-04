#include "plansys2_actions_cost/path_length.hpp"
#include "plansys2_actions_cost/path_smoothness.hpp"

#include "plansys2_msgs/msg/action_cost.hpp"
#include "nav_msgs/msg/path.hpp"
#include <iostream>

plansys2_msgs::msg::ActionCost::SharedPtr generic_function_that_uses_the_cost_function(plansys2_actions_cost::cost_function_t cost_function)
{
    std::cout << "OK" << std::endl;
    return cost_function();
}


int main()
{
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

    return 0;
}

