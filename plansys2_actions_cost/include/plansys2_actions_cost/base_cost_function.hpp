#ifndef PLANSYS2_ACTIONS_COST__BASE_COST_HPP_
#define PLANSYS2_ACTIONS_COST__BASE_COST_HPP_

#include <functional>
#include "plansys2_msgs/msg/action_cost.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

namespace plansys2_actions_cost
{
    class BaseCostFunction
    {
    public:
        explicit BaseCostFunction();

        virtual plansys2_msgs::msg::ActionCost cost_function() = 0;

        virtual ~BaseCostFunction() {};
    };

    // using ActionCostPtr = plansys2_msgs::msg::ActionCost::SharedPtr;
    // double compute_move_action_cost(BaseCostFunction cost_function, plansys2::ActionExecutorClient::Ptr action_executor_client_ptr)
    // {
    //     //Action Call ...
    //     action_executor_client_ptr->set_action_cost()
    // }

} // namespace plansys_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__BASE_COST_HPP_

