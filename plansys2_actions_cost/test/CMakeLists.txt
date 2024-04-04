ament_add_gtest(cost_functions_test cost_functions_test.cpp)
target_link_libraries(cost_functions_test
  ${PROJECT_NAME}
)
ament_target_dependencies(cost_functions_test ${dependencies})
