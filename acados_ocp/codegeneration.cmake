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