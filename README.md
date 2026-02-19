<img src="./assets/logo.png" height=150 align="right">

# trajectory_optimization

This repository contains a ROS 2 trajectory optimization package for automated driving based on [acados](https://github.com/acados/acados) and the [planning_interfaces](https://github.com/ika-rwth-aachen/planning_interfaces).

### Features

- The `trajectory_optimization` package contains a node that solves a nonlinear OCP online using `acados`, and publishes a drivable trajectory.
- Supports two driving modes:
  - Ackermann: `trajectory_optimization_ackermann_node`
  - Rear-wheel steering (RWS): `trajectory_optimization_rws_node`
- Handles dynamic obstacles and route boundaries, alongside state, control, and vehicle-dynamics constraints.
- Supports timer-based execution (`optimization_frequency`) or callback-based execution (`run_as_callback`).
- Offers model switching via `model_name`, selecting among pre-generated `acados` solver variants.
- Published trajectories are visualizable via RViz plugins from `planning_interfaces`.

### How It Works

1. Subscribe to [ego state](https://github.com/ika-rwth-aachen/perception_interfaces/blob/main/perception_msgs/msg/EgoData.msg), [objects](https://github.com/ika-rwth-aachen/perception_interfaces/blob/main/perception_msgs/msg/ObjectList.msg), [reference trajectory](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/trajectory_planning_msgs/msg/Trajectory.msg), and [route](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/route_planning_msgs/msg/Route.msg).
2. Transform inputs into the configured vehicle frame.
3. Assemble OCP parameters (cost weights/params, reference trajectory converted to path samples with boundary distances, dynamic stage weights, and obstacle circles).
4. Initialize the solver state either from current ego state (high-level stabilization) or from the previous valid trajectory with threshold-based fallback to ego state (bi-level stabilization).
5. Solve OCP with `acados` and publish a [drivable trajectory](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/trajectory_planning_msgs/msg/Trajectory.msg).
6. Optionally publish visualization markers for debug.

## Repository Packages

| Package | Purpose |
| --- | --- |
| `trajectory_optimization` | Runtime ROS 2 node that solves and publishes optimized trajectories. |
| `acados_ocp` | Generates `acados` C code/solvers for configured models and exports headers/libs. |
| `demo_trajectory_pub` | Test node that publishes synthetic reference trajectory, ego data, and object list. |

## Build and Run

The packages are best used inside a Docker container. 

You may use the provided container image: `TODO`.

Or build the container yourself by following these steps:

1. Add the [docker-ros](https://github.com/ika-rwth-aachen/docker-ros) repository as a submodule:
    ```bash
    git submodule add https://github.com/ika-rwth-aachen/docker-ros.git docker/docker-ros
    ```

2. Build the Docker image:
    ```bash
    BASE_IMAGE="rwthika/ros2:jazzy" \
    ENABLE_RECURSIVE_VCS_IMPORT="false" \
    COMMAND="ros2 launch trajectory_optimization trajectory_optimization.launch.py driving_mode:=ackermann" \
    IMAGE="trajectory_optimization:local" \
    ./docker/docker-ros/scripts/build.sh
    ```

Run the container and node by following these steps:

1. Run and attach to the downloaded container:
    ```bash
    docker run --rm -it TODO bash
    ```
    or run and attach to the locally built image:
    ```bash
    docker run --rm -it trajectory_optimization:local bash
    ```

2. Inside the container, source the workspace and, e.g., run the node:
    ```bash
    source install/setup.bash
    ros2 launch trajectory_optimization trajectory_optimization.launch.py
    ```


## `trajectory_optimization` node

### Launch Arguments ###

| Argument | Type | Description |
| --- | --- | --- |
| `driving_mode` | `string` | Driving mode, either `ackermann` or `rws` (default: `ackermann`). |
| `name` | `string` | Node name (default: executable name for selected driving mode). |
| `namespace` | `string` | Node namespace (default: empty). |
| `params` | `string` | Path to parameter file (default: `trajectory_optimization/config/params.yml`). |
| `log_level` | `string` | ROS log level (`debug|info|warn|error|fatal`, default: `info`). |
| `use_sim_time` | `bool` | Use simulation clock (`true|false`, default: `false`). |
| `trace` | `bool` | Enable tracing (`true|false`, default: `false`). |
| `ego_data_topic` | `string` | Remap for ego data topic (default: `~/ego_data`). |
| `object_list_topic` | `string` | Remap for object list topic (default: `~/object_list`). |
| `reference_trajectory_topic` | `string` | Remap for reference trajectory topic (default: `~/reference_trajectory`). |
| `route_topic` | `string` | Remap for route topic (default: `~/route`). |
| `trajectory_topic` | `string` | Remap for output trajectory topic (default: `~/trajectory`). |
| `object_circles_topic` | `string` | Remap for obstacle circles visualization topic (default: `~/visualization/object_circles`). |
| `ego_circles_topic` | `string` | Remap for ego circles visualization topic (default: `~/visualization/ego_circles`). |
| `boundary_marker_topic` | `string` | Remap for boundaries visualization topic (default: `~/visualization/boundaries`). |

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ego_data` | `perception_msgs::msg::EgoData` | Current ego vehicle state. |
| `~/object_list` | `perception_msgs::msg::ObjectList` | Detected objects; used for obstacle constraints if enabled. |
| `~/reference_trajectory` | `trajectory_planning_msgs::msg::Trajectory` | Reference trajectory to follow. |
| `~/route` | `route_planning_msgs::msg::Route` | Route / boundaries used when boundary constraints are enabled. |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/trajectory` | `trajectory_planning_msgs::msg::Trajectory` | Optimized output trajectory. |
| `~/visualization/object_circles` | `visualization_msgs::msg::MarkerArray` | Obstacle circle approximation markers. |
| `~/visualization/ego_circles` | `visualization_msgs::msg::MarkerArray` | Ego footprint circle markers. |
| `~/visualization/boundaries` | `visualization_msgs::msg::MarkerArray` | Boundary/intersection debug markers. |

### Parameters

Common parameters:

| Parameter | Type | Description |
| --- | --- | --- |
| `vehicle_frame_id` | `string` | Local frame for OCP formulation (default: `base_link`). |
| `trajectory_frame_id` | `string` | Output trajectory frame (default: `base_link`). |
| `fixed_over_time_frame_id` | `string` | Frame treated as fixed over time for temporal transforms (default: `map`). |
| `ego_data_timeout` | `double` | Max age of ego data before solver run is skipped (default: `1.0`). |
| `model_name` | `string` | OCP model name (`karl`, `shuttle`, `shuttle_ackermann`, `taxi`; default: `karl`). |
| `optimization_frequency` | `double` | Planner frequency in Hz when timer mode is used (default: `10.0`). |
| `run_as_callback` | `bool` | Run on each reference trajectory callback instead of timer (default: `false`). |
| `n_shots` | `int` | Number of shooting intervals (default: `50`). |
| `optimization_horizon` | `double` | Optimization horizon in seconds (default: `1.0`). |
| `verbose` | `bool` | Enable additional solver logging (default: `false`). |
| `debug_visualization` | `bool` | Publish debug marker topics (default: `false`). |
| `cost_weights` | `double[]` | Cost-function weights vector. |
| `dynamic_weight` | `double` | Dynamic weighting factor. |
| `thw` | `double` | Time headway (s). |
| `d_min_obstacle_long` | `double` | Minimum longitudinal obstacle distance (m). |
| `d_min_obstacle_lat` | `double` | Minimum lateral obstacle distance (m). |
| `d_min_boundary_lat` | `double` | Minimum lateral boundary distance (m). |
| `standstill_threshold` | `double` | Standstill detection threshold (m/s). |
| `high_level_stabilization` | `bool` | Use ego state directly for initialization (default: `false`). |
| `add_x_init_to_ref` | `bool` | Prepend initial state to reference when reference starts in front of ego. |
| `consider_objects` | `int` | `0`: none, `1`: static, `2`: predicted (default: `2`). |
| `consider_boundaries` | `int` | `0`: none, `1`: suggested lane, `2`: including adjacent, `3`: drivable space (default: `1`). |
| `bi_level_dV` | `double` | Bi-level threshold for velocity difference (m/s). |
| `bi_level_dA` | `double` | Bi-level threshold for acceleration difference (m/s²). |
| `bi_level_dY` | `double` | Bi-level threshold for lateral offset (m). |
| `bi_level_dYaw` | `double` | Bi-level threshold for yaw difference (deg). |
| `init_as_ref` | `bool` | Initialize states from reference under specific conditions (default: `false`). |

Ackermann-specific parameters:

| Parameter | Type | Description |
| --- | --- | --- |
| `bi_level_dDelta` | `double` | Bi-level threshold for Ackermann steering angle difference (deg). |

RWS-specific parameters:

| Parameter | Type | Description |
| --- | --- | --- |
| `distance_front_axle` | `double` | Distance from CoG to front axle (m). |
| `distance_rear_axle` | `double` | Distance from CoG to rear axle (m). |
| `bi_level_dDelta_front` | `double` | Bi-level threshold for front steering difference (deg). |
| `bi_level_dDelta_rear` | `double` | Bi-level threshold for rear steering difference (deg). |

Example parameter files:

- `trajectory_optimization/config/example_params_ackermann.yml`
- `trajectory_optimization/config/example_params_rws.yml`

## Model Generation (`acados_ocp`)

`acados_ocp` handles `acados` solver code generation during build and exports generated solver headers/libraries for `trajectory_optimization`.

For full details (generation pipeline, available model configs, artifact layout, and how to add/change models), see:
- [`acados_ocp/README.md`](acados_ocp/README.md)

Important: after adding/changing a model or model config in `acados_ocp`, rebuild the workspace to regenerate solver code.

## Contact

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> **Trajectory optimization** is one of many research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your advanced driver assistance and automated driving efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***
