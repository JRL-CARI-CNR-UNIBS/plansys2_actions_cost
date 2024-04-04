#include <gtest/gtest.h>

#include "plansys2_actions_cost/path_length.hpp"
#include "plansys2_actions_cost/path_smoothness.hpp"

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

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
