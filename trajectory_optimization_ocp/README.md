# `trajectory_optimization_ocp`

Defines the OCP for trajectory optimization and generates the corresponding C code headers/libraries, which are then used by `trajectory_optimization`.

It is used by [`trajectory_optimization`](../README.md).

### What this package does

- Runs acados code generation during CMake configure/build via [`codegeneration.cmake`](codegeneration.cmake).
- Builds solver shared libraries (`.so`) from generated code.
- Installs generated headers and libraries so downstream ROS 2 packages can link against them.
- Exposes an interface target (`trajectory_optimization_ocp`) that downstream packages can `find_package(...)` and link.

### Generation pipeline

Generation logic is defined in [`codegeneration.cmake`](codegeneration.cmake):

1. Copy generator sources from `trajectory_optimization_ocp/` into the build directory.
2. Run [`generate_ocp.py`](trajectory_optimization_ocp/generate_ocp.py) once per model config:
   - `karl_params.yml`
   - `shuttle_params.yml`
3. Build generated acados solver code/libraries.
4. Install generated headers and `.so` files.

In normal use, no separate generation command is needed. After adding a new model or changing an existing model/config, rebuild the workspace to automatically rebuild `trajectory_optimization_ocp` and rerun this generation pipeline.

### Available model definitions

Model equations are defined in:

- [`trajectory_optimization_ocp/models/model_Ackermann.py`](trajectory_optimization_ocp/models/model_Ackermann.py)
- [`trajectory_optimization_ocp/models/model_RWS.py`](trajectory_optimization_ocp/models/model_RWS.py)

Model/config parameterizations are defined in:

- [`trajectory_optimization_ocp/config/karl_params.yml`](trajectory_optimization_ocp/config/karl_params.yml)
- [`trajectory_optimization_ocp/config/shuttle_params.yml`](trajectory_optimization_ocp/config/shuttle_params.yml)

### Forward-only dynamics

Both bicycle models use a discrete transition that detects stopping inside a
shooting interval. Motion is integrated only until the stopping time; negative
longitudinal acceleration demand then holds the vehicle at zero speed. Positive
jerk can release the brake and restart within the same interval. Steering can
continue changing at standstill. Moving segments use two RK4 steps with exact
longitudinal velocity at their evaluation points.

State/control dimensions are unchanged. The longitudinal acceleration state and
trajectory acceleration represent the acceleration/braking demand, including a
negative holding demand at standstill. Its rate remains the jerk control;
acceleration/jerk penalties and limits continue to penalize/limit that demand.
The effective motion acceleration is zero while a stopped vehicle is held.

The interval duration is appended to the existing stage parameters. The node
sets it from the runtime horizon and uses the same generated transition for its
warm start. Negative initial speed estimates are projected to zero, and the
published velocity removes negative roundoff within the feasibility tolerance.
Changing the model requires rebuilding both generated solvers and the nodes.

### Generated artifacts

During build/install, this package provides:

- Generated solver headers under `install/trajectory_optimization_ocp/include/trajectory_optimization_ocp/trajectory_optimization_ocp/...`
- Generated solver shared libraries under `install/trajectory_optimization_ocp/lib`

### How to use in another package

In a downstream package:

1. Add `trajectory_optimization_ocp` as a dependency in `package.xml`.
2. `find_package(trajectory_optimization_ocp REQUIRED)` in `CMakeLists.txt`.
3. Link your target against `trajectory_optimization_ocp` (and acados runtime as needed).

### Adding a new model/configuration

1. Add/update model and/or parameter config under `trajectory_optimization_ocp/`.
2. Add a corresponding generation call in [`codegeneration.cmake`](codegeneration.cmake).
3. Rebuild the workspace; generated headers/libs will be exported by this package.
4. If used from `trajectory_optimization`, also wire it into the model dispatch there.
