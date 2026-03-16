cmake_minimum_required(VERSION 3.20)
project(trajectory_optimization_ocp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include(codegeneration.cmake)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

file(GLOB ACADOS_LIBS ${CMAKE_CURRENT_BINARY_DIR}/c_generated_code/build/*.so)
target_link_libraries(${PROJECT_NAME} INTERFACE ${ACADOS_LIBS})

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