#ifndef PLANSYS2_ACTIONS_COST__COST_FUNCTION_GENERATOR_HPP_
#define PLANSYS2_ACTIONS_COST__COST_FUNCTION_GENERATOR_HPP_

#include <functional>
#include "plansys2_msgs/msg/action_cost.hpp"
// #include "plansys2_executor/ActionExecutorClient.hpp"

namespace plansys2_actions_cost
{
    using ActionCostPtr = plansys2_msgs::msg::ActionCost::SharedPtr;
    using cost_function_ret_t = ActionCostPtr;
    using cost_function_t = std::function<cost_function_ret_t()>;
    
    struct BaseAbstractClass {};

    template<typename... Args>
    struct PluginGenerator : BaseAbstractClass
    {
        cost_function_ret_t(*f_)(Args ...);
        
        PluginGenerator(cost_function_ret_t(*f)(Args ...) ) : f_(f) {};
        
        cost_function_t args_binder(Args ... args)
        {
            return std::bind(f_, std::forward<Args>(args)...);
        }
    };

    #define CUSTOM_PLUGIN_GENERATOR( ObjectName, FunctionPointer, ... )\
    struct ObjectName :  PluginGenerator<__VA_ARGS__>\
    {\
        ObjectName () : PluginGenerator(FunctionPointer) {};\
    };

} // namespace plansys_actions_cost

#endif  // PLANSYS2_ACTIONS_COST__COST_FUNCTION_GENERATOR_HPP_

