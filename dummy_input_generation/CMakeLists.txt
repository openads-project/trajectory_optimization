cmake_minimum_required(VERSION 3.8)
project(dummy_input_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(perception_msgs_utils REQUIRED)
find_package(trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs_utils REQUIRED)

set(TARGET_NAME dummy_input_generation_node_component)

add_library(${TARGET_NAME} SHARED src/dummy_input_generation_node.cpp)

rclcpp_components_register_node(${TARGET_NAME}
  PLUGIN "dummy_input_generation::DummyInputGenerationNode"
  EXECUTABLE dummy_input_generation_node
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  rclcpp
  rclcpp_components
  perception_msgs
  perception_msgs_utils
  trajectory_planning_msgs
  trajectory_planning_msgs_utils
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
