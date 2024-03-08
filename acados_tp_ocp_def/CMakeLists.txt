cmake_minimum_required(VERSION 3.8)
project(acados_tp_ocp_def)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(acados_vendor_ros2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

set(TARGET_NAME acados_tp_ocp_def)

# Set sources for model
set(MODEL_SRC
    src/trajectory_planning_model/trajectory_planning_impl_dae_fun.c
    src/trajectory_planning_model/trajectory_planning_impl_dae_fun_jac_x_xdot_z.c
    src/trajectory_planning_model/trajectory_planning_impl_dae_jac_x_xdot_u_z.c
)

# Set sources for optimal control problem - mostly CasADi exports
set(OCP_SRC
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_0_fun.c
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_0_fun_jac.c
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_0_fun_jac_hess.c
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_fun.c
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_fun_jac.c
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_fun_jac_hess.c
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_e_fun.c
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_e_fun_jac.c
    src/trajectory_planning_cost/trajectory_planning_cost_ext_cost_e_fun_jac_hess.c
    src/acados_solver_trajectory_planning.c
)

# Combine sources
set(SRC_FILES ${MODEL_SRC} ${OCP_SRC})

add_library(${TARGET_NAME} SHARED ${SRC_FILES})

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  acados_vendor_ros2
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
