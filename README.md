<img src="./assets/logo.png" height=150 align="right">

# Trajectory Optimization for Automated Driving

This repository contains a ROS 2 node for periodically solving a nonlinear optimal control problem (OCP) to generate optimized trajectories for automated driving. The goal of the OCP is to follow a "reference trajectory" while respecting the dynamics of a given vehicle model and optimizing with respect to a configured cost function, all while not violating defined constraints such as collision avoidance and road boundaries. This results in a "drivable trajectory" that fulfils all these requirements.

<!-- This repository contains a ROS 2 node, that periodically solves a nonlinear optimal control problem (OCP) with the goal of following a reference trajectory under defined conditions and constraints. The result of the OCP is a trajectory that respects the dynamics of a given vehicle model and is optimized with respect to a configured cost function, while also not violating defined constraints such as collision avoidance and road boundaries. -->

The open-source framework [acados](https://github.com/acados/acados) is used to define OCP and generate the libraries for solving it online in the ROS 2 node. Key features:
- **Vehicle models**: single-track model with ackermann steering (and optionally rear-wheel steering).
- **Cost function**: minimizing tracking error to reference trajectory, acceleration, jerk, and steering actuation (with configurable weights).
- **Constraints**: dynamic obstacles, route boundaries, state/control/dynamics constraints.

The ROS 2 node uses the open-source ROS 2 message definitions [perception_interfaces](https://github.com/ika-rwth-aachen/perception_interfaces) and [planning_interfaces](https://github.com/ika-rwth-aachen/planning_interfaces) for all inputs and outputs, making it easy to integrate into a larger ROS 2-based system. The node is designed to be flexible and configurable, with support for different driving modes, model variants, and execution modes.

## Repository Packages

> For further details see the respective package README files.

| Package | Purpose |
| --- | --- |
| [trajectory_optimization](trajectory_optimization/README.md) | Runtime ROS 2 node that periodically solves the OCP and publishes optimized trajectories. |
| [acados_ocp](acados_ocp/README.md) | Defines the OCP and generates the corresponding C code headers/libraries, which are then used by `trajectory_optimization`. |
| [dummy_input_generation](dummy_input_generation/README.md) | Test node that generates and publishes synthetic inputs for the `trajectory_optimization`. |

## Build and Run

The packages are best used inside a Docker container.

The Docker container of this repository is automatically build and published via the CI pipeline.

```bash
docker pull TODO
```

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

## Contact

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> **Trajectory optimization** is one of many research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your advanced driver assistance and automated driving efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***
