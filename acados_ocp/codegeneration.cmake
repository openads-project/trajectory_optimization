## Generate OCP Model ##
file(GLOB ACADOS_FILES "trajectory_optimization_def/*.py" "trajectory_optimization_def/*.yml")

set(ACADOS_BUILD_FILES)
foreach(input_file ${ACADOS_FILES})
    # Extract name of the file for generate path of the file in the build tree
    get_filename_component(input_file_name ${input_file} NAME)
    # Path to the file created by copy
    set(build_input_file ${CMAKE_CURRENT_BINARY_DIR}/${input_file_name})
    # Copy file
    configure_file(${input_file} ${build_input_file} COPYONLY)
    # Add name of created file into the list
    list(APPEND ACADOS_BUILD_FILES ${build_input_file})
endforeach()

set(GENERATOR generate_ocp.py)

execute_process(
  COMMAND bash "-c" "python ${CMAKE_CURRENT_BINARY_DIR}/${GENERATOR}"
)