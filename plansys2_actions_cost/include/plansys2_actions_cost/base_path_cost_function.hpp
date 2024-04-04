/*
#ifndef PLANSYS2_ACTIONS_COST__BASE_PATH_COST_HPP_
#define PLANSYS2_ACTIONS_COST__BASE_PATH_COST_HPP_

#include "plansys2_msgs/msg/action_cost.hpp"
#include "plansys2_actions_cost/base_cost_function.hpp"

#include "nav_msgs/msg/path.hpp"

namespace plansys2_actions_cost
{
    class BasePathCostFunction : public BaseCostFunction
    {
    public:
        explicit BasePathCostFunction(const nav_msgs::msg::Path::SharedPtr& path) { path_= path; }
        // explicit init(const nav_msgs::msg::Path::SharedPtr& path) { path_=path; }
        // template <typename T>
        // virtual plansys2_msgs::msg::ActionCost cost_function(const T& val) override {throw std::runtime_error("The type is not implemented. Aborted");};

        virtual ~BasePathCostFunction() {};
    protected:
        nav_msgs::msg::Path::SharedPtr path_ = nullptr;
    };
    
} // namespace plansys_actions_cost


#endif  // PLANSYS2_ACTIONS_COST__BASE_PATH_COST_HPP_

*/