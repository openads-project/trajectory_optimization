# trajectory_optimization

This package provides a ROS 2 node for solving the OCPs defined [here](../acados_ocp/). The OCP is based on [ACADOS](https://docs.acados.org/index.html) and is used for trajectory planning and optimization in the context of autonomous driving. This node provides the runtime integration of the OCP into ika's AD-Stack and acts as trajectory planner for the ego vehicle.

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ego_data` | `perception_msgs::msg::EgoData` | Contains the current state of the ego vehicle. |
| `~/object_list` | `perception_msgs::msg::ObjectList` | Contains a list of detected objects. Used in OCP for collision avoidance. |
| `~/reference_trajectory` | `trajectory_planning_msgs::msg::Trajectory` | Contains the reference trajectory, which the ego vehicle should follow. |
| `~/route` | `route_planning_msgs::msg::Route` | Contains route and drivable space boundaries. (Currently unused in OCP) |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/trajectory` | `trajectory_planning_msgs::msg::Trajectory` | Contains the planned trajectory for the ego vehicle. Resluting from the OCP. |
| `~/visualization/object_circles` | `visualization_msgs::msg::MarkerArray` | Contains RViz markers for direct visualization of the OCP inputs (objects). |

### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `vehicle_frame_id` | `std::string` | Frame ID of local vehicle frame (the ocp is defined in this frame). Default: `base_link` |
| `trajectory_frame_id` | `std::string` | Frame ID of output trajectory. Default: `base_link` |
| `fixed_over_time_frame_id` | `std::string` | Frame ID of frame that is fixed over time for finding temporal transforms. Default: `map` |
| `model_name` | `std::string` | Name of the model to be used for trajectory optimization [passat_cc, auto_shuttle]. Default: `passat_cc` |
| `optimization_frequency` | `double` | Optimization Frequency in Hz. Default: `10.0` |
| `n_shots` | `int` | Number of shooting intervals in optimization horizon. Default: `50` |
| `optimization_horizon` | `double` | Optimization Horizon in seconds. Default: `1.0` |
| `verbose` | `bool` | Print solver statistics. Default: `false` |
| `debug_visualization` | `bool` | Publish debug visualization markers (e.g. obstacle circles). Default: `false` |
| `wheelbase` | `double` | Wheelbase of the vehicle [m] (should be aligned with the OCP). Default: `2.711` |
| `cost_weights` | `std::vector<double>` | Cost function weights. Default: all elements `1.0` |
| `dynamic_weight` | `double` | Dynamic weight alpha. Default: `1.0` |
| `thw` | `double` | Time headway to front vehicle. Default: `2.0` |
| `d_min_obstacle_long` | `double` | Minimum distance to keep to obstacle in longitudinal direction [m]. Default: `5.0` |
| `d_min_obstacle_lat` | `double` | Minimum distance to keep to obstacle in lateral direction [m]. Default: `0.5` |
| `standstill_threshold` | `double` | Threshold for standstill detection [m/s]. If all state velocities are below this threshold, publish standstill trajectory. Default: `0.45` |
| `high_level_stabilization` | `bool` | Use high-level stabilization strategy for init state (= init with current EgoData). Default: `false` |
| `use_prediction` | `bool` | Use obstacle predictions for optimization (True) or only static obstacles (False). Default: `false` |
| `bi_level_dV` | `double` | Threshold for bi-level stabilization: maximum velocity difference [m/s]. Default: `5.0` |
| `bi_level_dA` | `double` | Threshold for bi-level stabilization: maximum acceleration difference [m/s^2]. Default: `2.0` |
| `bi_level_dY` | `double` | Threshold for bi-level stabilization: maximum y-offset [m]. Default: `0.1` |
| `bi_level_dYaw` | `double` | Threshold for bi-level stabilization: maximum yaw difference [degree]. Default: `5.0` |
| `bi_level_dDelta` | `double` | Threshold for bi-level stabilization: maximum steering angle difference [degree]. Default: `90.0` |
| `init_as_ref` | `bool` | Boolean that enables initialization of trajectory states as reference states under certain set of conditions. Default: `false` |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| `latest` | run image |
| `slim` | minimal run image |
| `latest-dev` | development image |

### Default Command

```bash
ros2 launch trajectory_optimization trajectory_optimization_node.launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `trajectory_optimization` | `trajectory_optimization_node.launch.py` | `/docker-ros/ws/install/trajectory_optimization/share/trajectory_optimization/launch/` | Default launch file for the trajectory optimization node. Remapping and other launch parameters can be set here. |

### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `trajectory_optimization` | `params.yml` | `install/trajectory_optimization/share/trajectory_optimization/config/` | Example configuration file for the trajectory optimization node (these are not the default parameters). |

### Usage of different models

The [ocp_model_handler.hpp](./include/trajectory_optimization/ocp_model_handler.hpp) contains wrapper functions to handle different OCP models / parameterizations within this node (i.e. `passat_cc` and `auto_shuttle`). The `model_name` parameter can be used to switch between these models. In case of adding a new model, following steps are necessary:
1. Add the new model / parameterization in the `acados_ocp` package, as described [here](../acados_ocp/README.md).
2. Add the new model / parameterization in the [ocp_model_handler.hpp](./include/trajectory_optimization/ocp_model_handler.hpp#L28-L35) and extend the wrapper functions accordingly.