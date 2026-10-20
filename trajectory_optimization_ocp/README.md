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

### Object constraints

Both vehicle OCPs separate desired clearance from physical collision avoidance:

- Desired clearance: the ego box expanded by THW/`d_min_object_long` to the front,
  `d_min_object_lat` to the sides and a small rear margin. `enable_object_slack` softens
  this block using `object_slack_weights`; without slacks it stays hard.
- Physical separation: the unexpanded ego and object boxes must be separated by the build
  parameter `d_min_object_collision` (default 0.10 m). This block follows the clearance
  block and never receives slacks.

Both blocks use the conservative smooth SAT margin and apply at intermediate and terminal
nodes, not retroactively at the fixed initial state. Node constraints alone do not guarantee
collision avoidance between nodes or under incorrect predictions. `thw`, `d_min_object_long`
and `d_min_object_lat` are runtime parameters; changing `d_min_object_collision` or the slack
weights requires rebuilding the generated solver. Generation errors fail the build instead of
reusing stale solver artifacts.

### Forward-only dynamics

Both bicycle models use `v_dot = max(a, -v / tau_stop)` with `tau_stop = 0.05 s`.
Braking is unchanged until `v < -a * tau_stop`; below that speed, velocity decays
exponentially toward zero. Positive demand still permits restart. With Karl's
configured braking limit, the approximation applies below 0.225 m/s.

The acceleration state and published trajectory acceleration represent demand,
with `a_dot = j`. Longitudinal acceleration/jerk costs and limits apply to demand;
effective acceleration and jerk can differ near standstill. State/control
dimensions and stage parameters are unchanged.

The OCP and warm-start rollout use ERK/RK4 with four substeps per interval
(25 ms by default). Other step sizes require validation; ERK alone does not
guarantee nonnegative speed. The node clamps negative initial speed and, at zero
speed, negative initial acceleration. Cached controls are cleared when the previous
optimized trajectory indicates standstill. Published speeds are clamped to zero after feasibility
validation to remove negative roundoff. Model changes require rebuilding both
generated solvers and nodes.

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
