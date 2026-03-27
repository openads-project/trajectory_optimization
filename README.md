# trajectory_optimization <img src="./assets/logo.png" height=40 align="right">

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/trajectory_optimization/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/trajectory_optimization"/></a>
  <a href="https://github.com/openads-project/trajectory_optimization/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/trajectory_optimization"/></a>
  <br>
  <a href="https://github.com/openads-project/trajectory_optimization/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/trajectory_optimization/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/trajectory_optimization"><img src="https://github.com/openads-project/trajectory_optimization/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/trajectory_optimization/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/trajectory_optimization/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

This repository contains a ROS 2 node for periodically solving a nonlinear optimal control problem (OCP) to generate optimized trajectories for automated driving. The goal of the OCP is to follow a "reference trajectory" while respecting the dynamics of a given vehicle model and optimizing with respect to a configured cost function, all while not violating defined constraints such as collision avoidance and road boundaries. This results in a "drivable trajectory" that fulfils all these requirements.

The open-source framework [acados](https://github.com/acados/acados) is used to define OCP and generate the libraries for solving it online in the ROS 2 node. Key features:
- **Vehicle models**: single-track model with ackermann steering (and optionally rear-wheel steering).
- **Cost function**: minimizing tracking error to reference trajectory, acceleration, jerk, and steering actuation (with configurable weights).
- **Constraints**: dynamic obstacles, route boundaries, state/control/dynamics constraints.

The ROS 2 node uses the open-source ROS 2 message definitions [perception_interfaces](https://github.com/ika-rwth-aachen/perception_interfaces) and [planning_interfaces](https://github.com/ika-rwth-aachen/planning_interfaces) for all inputs and outputs, making it easy to integrate into a larger ROS 2-based system. The node is designed to be flexible and configurable, with support for different driving modes, model variants, and execution modes.

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>🧑‍💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>

> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Stack*.


<!-- <img src="TODO: teaser image/gif" width=800> -->


## 🚀 Quick Start

Run the ready-made demo setup from [`docker/demo`](./docker/demo), which starts the trajectory optimization together with the dummy input generation, RViz and RQt. You can use the dynamic reconfigure options in RQt to change the parameters of the trajectory optimization and see the effect on the generated trajectories in RViz.

1. On a local Linux desktop, allow Docker containers to open X11 windows.
    ```bash
    xhost +local:
    ```
1. Start the demo stack.
    ```bash
    cd docker/demo
    docker compose up
    ```
1. Stop the stack with `Ctrl+C` and clean up the containers.
    ```bash
    docker compose down
    ```

## 🧑‍💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/openads-project/trajectory_optimization.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd trajectory_optimization
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Build

> *Ctrl+Shift+B*

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

For further details see the respective package README files and the [Documentation](https://openads-project.github.io/trajectory_optimization).

| Package | Purpose |
| --- | --- |
| [dummy_input_generation](dummy_input_generation/README.md) | Test node that generates and publishes synthetic inputs for the `trajectory_optimization`. |
| [trajectory_optimization](trajectory_optimization/README.md) | Runtime ROS 2 node that periodically solves the OCP and publishes optimized trajectories. |
| [trajectory_optimization_ocp](trajectory_optimization_ocp/README.md) | Defines the OCP and generates the corresponding C code headers/libraries, which are then used by `trajectory_optimization`. |

## ⚖️ Licensing

- The source code in this repository is licensed under Apache-2.0. See [LICENSE](LICENSE).
- Docker images built from this repository also contain third-party software with its own license terms.
- `acados` and its bundled dependencies ship license files in the container under `/opt/acados/LICENSE` and `/opt/acados/external/*/LICENSE*`.
- `CasADi` is used for code generation and is distributed under LGPL-3.0-or-later. In the current container, its package metadata is available under `/usr/local/lib/python3.12/dist-packages/casadi-3.7.2.dist-info/METADATA`.

## 🙏 Acknowledgements

This work is accomplished within the projects AIthena and autotech.*agil*. We acknowledge the financial support for the projects by
- the *European Union’s Horizon Europe Research and Innovation Programme* :eu: under Grant Agreement No 101076754 for AIthena,
- and the *Federal Ministry of Education and Research of Germany (BMBF)* :de: for AUTOtech.*agil* (FKZ 01IS22088A).
