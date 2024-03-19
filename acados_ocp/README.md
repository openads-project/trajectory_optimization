# acados_ocp

ROS 2 wrapper package for simple generation and usage of acados:
- `codegeneration.cmake` (included in `CMakeLists.txt`) -> automatic generation and build of acados c-code (incl. `.so`-files) via CMake with use of Python files in `trajectory_optimization_def`
- package `acados_ocp` includes generated header-files and links against generated acados libraries. To use the generated code and libs in a ROS node, it is now simple to add this package "normally" in the `CMakeLists.txt` and `package.xml`.