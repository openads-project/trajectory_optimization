cmake_minimum_required(VERSION 3.8)
project(trajectory_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(acados_vendor_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

## Generate OCP Model ##

file(GLOB ACADOS_FILES "acados_ocp/*.py")
file(COPY ${ACADOS_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(GENERATOR generate_ocp.py)

add_custom_target(acados_model ALL
  COMMAND python ${CMAKE_CURRENT_BINARY_DIR}/${GENERATOR}
  COMMENT "Build ${CMAKE_CURRENT_BINARY_DIR}/${GENERATOR}"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/c_generated_code/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/c_generated_code/build/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so"
)
####################

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

target_link_libraries(${TARGET_NAME} ${LIB_ACADOS_OCP_SOLVER} ${LIB_ACADOS_SOLVER})

ament_target_dependencies(${TARGET_NAME}
  acados_vendor_ros2
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
