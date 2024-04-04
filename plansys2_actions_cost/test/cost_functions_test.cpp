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
// // Test case for path_smoothness
// TEST(CostFunctionTest, PathSmoothnessTest)
// {
//     // Create an instance of the class being tested
//     plansys2_actions_cost::PathSmoothness path_smoothness;

//     // Create a sample path
//     nav_msgs::msg::Path path;
//     geometry_msgs::msg::PoseStamped pose1, pose2;
//     pose1.pose.position.x = 0.0;
//     pose1.pose.position.y = 0.0;
//     pose2.pose.position.x = 1.0;
//     pose2.pose.position.y = 1.0;
//     path.poses.push_back(pose1);
//     path.poses.push_back(pose2);

//     // Calculate the expected smoothness
//     double expected_smoothness = 0.0; // TODO: Calculate the expected smoothness

//     // Call the function being tested
//     double actual_smoothness = path_smoothness.path_smoothness(path);

//     // Check if the actual smoothness matches the expected smoothness
//     EXPECT_DOUBLE_EQ(actual_smoothness, expected_smoothness);
// }

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}