cmake_minimum_required(VERSION 3.20)
project(acados_ocp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(acados_vendor_ros2 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

message("acados_vendor_ros2 install space: ${CMAKE_INSTALL_PREFIX}")

# Download t_renderer
set(FILE_URL "https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux")
set(DESTINATION_PATH "/docker-ros/ws/install/acados_vendor_ros2/opt/acados_vendor_ros2/bin/t_renderer")

file(DOWNLOAD ${FILE_URL} ${DESTINATION_PATH}
     TIMEOUT 60
     SHOW_PROGRESS
)
# Make the file executable
file(CHMOD ${DESTINATION_PATH} PERMISSIONS WORLD_EXECUTE OWNER_EXECUTE OWNER_READ OWNER_WRITE GROUP_EXECUTE GROUP_READ GROUP_WRITE)

## Generate OCP Model ##

file(GLOB ACADOS_FILES "trajectory_optimization_def/*.py")
file(COPY ${ACADOS_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(GENERATOR generate_ocp.py)

add_custom_target(acados_model ALL
  COMMAND python ${CMAKE_CURRENT_BINARY_DIR}/${GENERATOR}
  COMMENT "Build ${CMAKE_CURRENT_BINARY_DIR}/${GENERATOR}"
)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/c_generated_code/
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/c_generated_code/build/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so"
)

ament_package()