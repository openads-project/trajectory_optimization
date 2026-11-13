cmake_minimum_required(VERSION 3.8)
project(trajectory_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(acados REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(perception_msgs_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(route_planning_msgs REQUIRED)
find_package(route_planning_msgs_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_perception_msgs REQUIRED)
find_package(tf2_route_planning_msgs REQUIRED)
find_package(tf2_trajectory_planning_msgs REQUIRED)
find_package(tracetools REQUIRED)
find_package(trajectory_optimization_ocp REQUIRED)
find_package(trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(TARGET_NAME trajectory_optimization_node_component)

add_library(${TARGET_NAME} SHARED 
  src/callbacks.cpp
  src/performance_logger.cpp
  src/trajectory_optimization_node.cpp
  src/trajectory_optimization_ackermann.cpp
  src/trajectory_optimization_rws.cpp
  src/utils.cpp
)

target_link_libraries(${TARGET_NAME}
  acados
)

rclcpp_components_register_nodes(${TARGET_NAME}
  "trajectory_optimization::TrajectoryOptimizationAckermannNode"
  "trajectory_optimization::TrajectoryOptimizationRWSNode"
)

foreach(model ackermann rws)
  add_executable(trajectory_optimization_${model}_node src/main_${model}.cpp)
  target_link_libraries(trajectory_optimization_${model}_node ${TARGET_NAME})
  ament_target_dependencies(trajectory_optimization_${model}_node rclcpp acados)
  install(TARGETS trajectory_optimization_${model}_node DESTINATION lib/${PROJECT_NAME})
endforeach()

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  acados
  perception_msgs
  perception_msgs_utils
  rclcpp
  rclcpp_components
  route_planning_msgs
  route_planning_msgs_utils
  std_msgs
  tf2
  tf2_perception_msgs
  tf2_route_planning_msgs
  tf2_trajectory_planning_msgs
  tracetools
  trajectory_optimization_ocp
  trajectory_planning_msgs
  trajectory_planning_msgs_utils
  visualization_msgs
)

install(TARGETS ${TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_performance_logger test/test_performance_logger.cpp)
  target_link_libraries(test_performance_logger ${TARGET_NAME})
  ament_target_dependencies(test_performance_logger acados)
  ament_add_gtest(test_input_callbacks test/test_input_callbacks.cpp)
  target_link_libraries(test_input_callbacks ${TARGET_NAME})
  ament_target_dependencies(test_input_callbacks rclcpp acados)
  target_compile_definitions(test_input_callbacks PRIVATE
    TEST_PARAMETER_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/example_params_ackermann.yml")
  set_tests_properties(test_input_callbacks PROPERTIES
    ENVIRONMENT "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_DOMAIN_ID=83")
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/format/.clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/.clang-tidy)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/ament_flake8.ini)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
