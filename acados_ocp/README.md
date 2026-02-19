# acados_ocp

`acados_ocp` is a ROS 2 wrapper package that generates acados OCP solver code at build time and exports it as a normal CMake/ament dependency.

It is used by [`trajectory_optimization`](../README.md).

## What this package does

- Runs acados code generation during CMake configure/build via [`codegeneration.cmake`](codegeneration.cmake).
- Builds solver shared libraries (`.so`) from generated code.
- Installs generated headers and libraries so downstream ROS 2 packages can link against them.
- Exposes an interface target (`acados_ocp`) that downstream packages can `find_package(...)` and link.

## Generation pipeline

Generation logic is defined in [`codegeneration.cmake`](codegeneration.cmake):

1. Copy generator sources from `trajectory_optimization_def/` into the build directory.
2. Run [`generate_ocp.py`](trajectory_optimization_def/generate_ocp.py) once per model config:
   - `karl_params.yml`
   - `shuttle_params.yml`
   - `shuttle_ackermann_params.yml`
   - `taxi_params.yml`
3. Build generated acados solver code/libraries.
4. Install generated headers and `.so` files.

In normal use, no separate generation command is needed. After adding a new model or changing an existing model/config, rebuild the workspace to automatically rebuild `acados_ocp` and rerun this generation pipeline.

## Available model definitions

Model equations are defined in:

- [`trajectory_optimization_def/models/model_Ackermann.py`](trajectory_optimization_def/models/model_Ackermann.py)
- [`trajectory_optimization_def/models/model_RWS.py`](trajectory_optimization_def/models/model_RWS.py)

Model/config parameterizations are defined in:

- [`trajectory_optimization_def/config/karl_params.yml`](trajectory_optimization_def/config/karl_params.yml)
- [`trajectory_optimization_def/config/shuttle_params.yml`](trajectory_optimization_def/config/shuttle_params.yml)
- [`trajectory_optimization_def/config/shuttle_ackermann_params.yml`](trajectory_optimization_def/config/shuttle_ackermann_params.yml)
- [`trajectory_optimization_def/config/taxi_params.yml`](trajectory_optimization_def/config/taxi_params.yml)

## Generated artifacts

During build/install, this package provides:

- Generated solver headers under `install/acados_ocp/include/acados_ocp/acados_ocp/...`
- Generated solver shared libraries under `install/acados_ocp/lib`

## How to use in another package

In a downstream package:

1. Add `acados_ocp` as a dependency in `package.xml`.
2. `find_package(acados_ocp REQUIRED)` in `CMakeLists.txt`.
3. Link your target against `acados_ocp` (and acados runtime as needed).

## Adding a new model/configuration

1. Add/update model and/or parameter config under `trajectory_optimization_def/`.
2. Add a corresponding generation call in [`codegeneration.cmake`](codegeneration.cmake).
3. Rebuild the workspace; generated headers/libs will be exported by this package.
4. If used from `trajectory_optimization`, also wire it into the model dispatch there.
