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

Both bicycle models keep continuous dynamics with ERK integration. Their
longitudinal motion uses `v_dot = max(a, -v / tau_stop)`, with the build-time
time constant `tau_stop = 0.05` s defined directly in both model files. Normal braking
is unchanged until `v < -a * tau_stop`. Below that speed, braking decreases and
velocity approaches zero exponentially rather than reaching exactly zero.
At zero speed, negative demand holds the vehicle; positive demand allows restart.
For constant braking at 4 m/s², the change starts below 0.2 m/s and adds 5 mm to
the continuous-model stopping distance compared with an exact stop.

State/control dimensions and stage parameters are unchanged. The longitudinal
acceleration state and trajectory acceleration represent acceleration/braking
demand. Its rate remains the jerk control, and acceleration/jerk penalties and
limits apply to that demand. Near standstill, effective motion acceleration can
differ from the demand.

The OCP and existing warm-start rollout both use four RK4 steps per shooting
interval (25 ms for the default 5 s horizon and 50 intervals). A larger runtime
interval needs integration accuracy checked against the stopping time constant;
ERK does not provide a general nonnegative-speed guarantee for arbitrary step
sizes. Negative initial speed estimates are projected to zero, and published
velocity removes negative roundoff within the feasibility tolerance. Existing
standstill detection remains in use. Changing the model requires rebuilding
both generated solvers and the nodes.

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
