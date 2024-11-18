# acados_ocp

ROS 2 wrapper package for simple generation and usage of acados:
- `codegeneration.cmake` (included in `CMakeLists.txt`) -> automatic generation and build of acados c-code (incl. `.so`-files) via CMake with use of Python files in `trajectory_optimization_def`.
- package `acados_ocp` includes generated header-files and links against generated acados libraries. To use the generated code and libs in a ROS node, it is now simple to add this package "normally" in the `CMakeLists.txt` and `package.xml`.
- used in [`trajectory_optimization`](../trajectory_optimization) package.

## Usage of different models

- currently implemented is a single track model with `x = [x, y, s, v, a_lon, psi, delta]` and `u = [j_lon, alpha]`. Also see [model.py](trajectory_optimization_def/model.py) for the model definition.
- to use differente paramteriaztions of this model, the following steps are necessary:
  1. add a new parameter file in `trajectory_optimization_def` like [passat_cc_params.yml](trajectory_optimization_def/passat_cc_params.yml).
  2. add the corresponding model generation lines in [codegeneration.cmake](codegeneration.cmake#L23-L26)
  3. afterwards all models are build automatically and can be used for example in the `trajectory_optimization` package.