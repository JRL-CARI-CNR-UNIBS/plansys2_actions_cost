#include "plansys2_actions_cost/cost_function_path_length.hpp"
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
    nav_msgs::msg::Path::SharedPtr test_path_ptr(new nav_msgs::msg::Path);
    test_path_ptr->poses.resize(2);

    // Assegnare valori casuali alle pose del path di test
    test_path_ptr->poses[0].pose.position.x = 0.0;
    test_path_ptr->poses[0].pose.position.y = 0.0;
    test_path_ptr->poses[1].pose.position.x = 1.0;
    test_path_ptr->poses[1].pose.position.y = 1.0;


    // ESEMPIO NON PLUGINLIB
    //    Definisco l'oggetto che voglio usare
    plansys2_actions_cost::PathLength2 g;
    
    // // // faccio il bind degli argomenti. Il ref serve se ho usato il ref negli argomenti
    auto h = g.args_binder(std::ref(test_path_ptr));
    std::cout << generic_function_that_uses_the_cost_function(h)->nominal_cost << std::endl;

    return 0;
}

