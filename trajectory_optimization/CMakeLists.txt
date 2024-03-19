cmake_minimum_required(VERSION 3.8)
project(trajectory_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(acados_vendor_ros2 REQUIRED)
find_package(acados_ocp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

set(TARGET_NAME trajectory_optimization_node_component)

add_library(${TARGET_NAME} SHARED src/trajectory_optimization_node.cpp)

rclcpp_components_register_node(${TARGET_NAME}
  PLUGIN "trajectory_optimization::TrajectoryOptimizationNode"
  EXECUTABLE trajectory_optimization_node
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  acados_vendor_ros2
  acados_ocp
  rclcpp
  rclcpp_components
  std_msgs
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

ament_package()
