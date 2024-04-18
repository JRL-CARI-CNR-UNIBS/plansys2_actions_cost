cmake_minimum_required(VERSION 3.8)
project(plansys2_actions_cost)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    plansys2_msgs
    nav2_msgs
    nav2_util
    nav2_costmap_2d
    plansys2_executor
    tf2_geometry_msgs
    pluginlib
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/cost_functions/path_length.cpp
  src/cost_functions/path_smoothness.cpp
  src/cost_functions/path_cost.cpp
  src/plansys2_actions_cost/move_action_cost_base.cpp
  src/plansys2_actions_cost/move_action_cost_length.cpp
  src/plansys2_actions_cost/move_action_cost_smoothness.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(cost_test
  src/main.cpp
)
ament_target_dependencies(cost_test
  ${dependencies}
)

pluginlib_export_plugin_description_file(plansys2_actions_cost actions_plugins.xml)

target_link_libraries(cost_test ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  cost_test
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
