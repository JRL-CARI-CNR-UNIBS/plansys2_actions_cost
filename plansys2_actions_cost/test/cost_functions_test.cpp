#include <gtest/gtest.h>

#include "plansys2_actions_cost/path_length.hpp"
#include "plansys2_actions_cost/path_smoothness.hpp"
#include "plansys2_actions_cost/path_cost.hpp"

#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Test case for path_length
TEST(CostFunctionTest, PathLengthTest)
{
    plansys2_actions_cost::PathLength path_length;

    nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);

    geometry_msgs::msg::PoseStamped pose1, pose2;
    pose1.pose.position.x = 0.0;
    pose1.pose.position.y = 0.0;
    pose2.pose.position.x = 1.0;
    pose2.pose.position.y = 1.0;
    path_ptr->poses.push_back(pose1);
    path_ptr->poses.push_back(pose2);

    double expected_length = sqrt(2.0);
    auto path_length_cost_function = path_length.args_binder(std::ref(path_ptr));

    double actual_length = path_length_cost_function()->nominal_cost;

    EXPECT_DOUBLE_EQ(actual_length, expected_length);
}
// Test case for path_length with multiple poses
TEST(CostFunctionTest, PathLengthMultipleTest)
{
    plansys2_actions_cost::PathLength path_length;

    nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);

    geometry_msgs::msg::PoseStamped pose1, pose2, pose3, pose4;
    pose1.pose.position.x = 0.0;
    pose1.pose.position.y = 0.0;
    pose2.pose.position.x = 1.0;
    pose2.pose.position.y = 1.0;
    pose3.pose.position.x = 2.0;
    pose3.pose.position.y = 1.0;
    pose4.pose.position.x = 3.0;
    pose4.pose.position.y = 0.0;

    path_ptr->poses.push_back(pose1);
    path_ptr->poses.push_back(pose2);
    path_ptr->poses.push_back(pose3);
    path_ptr->poses.push_back(pose4);

    double expected_length = sqrt(2.0) + sqrt(2.0) + 1.0;

    auto path_length_cost_function = path_length.args_binder(std::ref(path_ptr));

    double actual_length = path_length_cost_function()->nominal_cost;

    EXPECT_DOUBLE_EQ(actual_length, expected_length);
}
// Test case for path_smoothness
TEST(CostFunctionTest, PathSmoothnessTest)
{
    plansys2_actions_cost::PathSmoothness path_smoothness;
    nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);

    path_ptr->poses.resize(2);

    // Rotation of 10° around the z axis
    path_ptr->poses[0].pose.position.x = 1.0;
    path_ptr->poses[0].pose.position.y = 0.0;
    path_ptr->poses[0].pose.orientation.z = 0.08715574;  
    path_ptr->poses[0].pose.orientation.w = 0.9961947;  
    
    // Rotation of 30° around the z axis
    path_ptr->poses[1].pose.position.x = 2.0;
    path_ptr->poses[1].pose.position.y = 0.0;
    path_ptr->poses[1].pose.orientation.z = 0.25881905;  
    path_ptr->poses[1].pose.orientation.w = 0.96592583;  


    double expected_smoothness = 20.0 * (M_PI / 180.0);

    auto path_smoothness_cost_function = path_smoothness.args_binder(std::ref(path_ptr));

    double actual_smoothness = path_smoothness_cost_function()->nominal_cost;

    double tolerance = 1e-5;
    EXPECT_NEAR(actual_smoothness, expected_smoothness, tolerance);
    // EXPECT_DOUBLE_EQ(actual_smoothness, expected_smoothness);
}

// class CostFunctionTest_PathCostTest_Test : public ::testing::Test {
// protected:
//     void SetUp() override {
//         rclcpp::init(0, nullptr);  // Inizializza il nodo ROS

//         // Crea un nodo ROS
//         auto node = std::make_shared<rclcpp::Node>("test_node");

//         // Crea un WeakPtr utilizzando il nodo ROS
//         weak_node_ptr = node;
//     }

//     std::weak_ptr<rclcpp::Node> weak_node_ptr;
// };

// // Classe mock per il subscriber del costmap
// class MockCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber {
// public:
//     // Costruttore predefinito
//     MockCostmapSubscriber() = default;

//     // Costruttore con argomenti
//     MockCostmapSubscriber(const std::weak_ptr<rclcpp::Node>& wp, const std::string& str) 
//         : CostmapSubscriber(wp, str) {}  
// //     std::shared_ptr<nav2_costmap_2d::Costmap2D> getCostmap(){
// //     // Create a mock costmap with random values
// //     auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 0.05, 0, 0);
// //     for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
// //         for (unsigned int y = 0; y < costmap->getSizeInCellsY(); ++y) {
// //             costmap->setCost(x, y, rand() % 255); // Random cost values between 0 and 254
// //         }
// //     }
// //     return costmap;
// // }

// };

TEST(CostFunctionTest, PathCostTest)
{
    nav_msgs::msg::Path::SharedPtr path_ptr(new nav_msgs::msg::Path);
    path_ptr->poses.resize(2);

    
    path_ptr->poses[0].pose.position.x = 1.0;
    path_ptr->poses[0].pose.position.y = 0.0;

    path_ptr->poses[1].pose.position.x = 2.0;
    path_ptr->poses[1].pose.position.y = 0.0;

    auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");

    // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

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
    auto costmap_publisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
        node->shared_from_this(), costmap_to_send.get(), "", "/fake_cost_map", true);

    costmap_publisher->on_activate();
    costmap_publisher->publishCostmap();

    rclcpp::spin_some(node->get_node_base_interface());

    auto costmap = costmap_subscriber->getCostmap();

    double lambda_1 = 1;
    double lambda_2 = 0.5;
    double action_cost_lambda_1 = std::numeric_limits<double>::infinity();
    double action_cost_lambda_2 = std::numeric_limits<double>::infinity();
    if(costmap)
    {
        std::cerr << costmap->getSizeInCellsX() << std::endl;
        RCLCPP_INFO(node->get_logger(), "Costmap available");

        plansys2_actions_cost::PathCost path_cost;

        std::cerr << &path_cost << std::endl;
        std::cerr << &path_cost << std::endl;

        auto path_cost_function_lambda_1 = path_cost.args_binder(std::ref(path_ptr), std::ref(costmap), std::ref(lambda_1));
        auto path_cost_function_lambda_2 = path_cost.args_binder(std::ref(path_ptr), std::ref(costmap), std::ref(lambda_2));
        // std::cerr << &path_cost_function_lambda_1 << std::endl;
        // std::cerr << &path_cost_function_lambda_2 << std::endl;

        action_cost_lambda_1 = path_cost_function_lambda_1()->nominal_cost;
        action_cost_lambda_2 = path_cost_function_lambda_2()->nominal_cost;
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Costmap not available");
    }
        std::cerr << default_value * path_ptr->poses.size() << std::endl;

    ASSERT_NEAR(action_cost_lambda_1, default_value * path_ptr->poses.size(), 1e-5);
    ASSERT_NEAR(action_cost_lambda_2, default_value * 1 + default_value * std::pow(lambda_2, 1), 1e-5);
    
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    return RUN_ALL_TESTS();
}
