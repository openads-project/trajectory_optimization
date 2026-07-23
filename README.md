# trajectory_optimization

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/trajectory_optimization/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/trajectory_optimization"/></a>
  <a href="https://github.com/openads-project/trajectory_optimization/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/trajectory_optimization"/></a>
  <br>
  <a href="https://github.com/openads-project/trajectory_optimization/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/trajectory_optimization/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/trajectory_optimization/actions/workflows/compose-oci.yml"><img src="https://github.com/openads-project/trajectory_optimization/actions/workflows/compose-oci.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/trajectory_optimization"><img src="https://github.com/openads-project/trajectory_optimization/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/trajectory_optimization/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/trajectory_optimization/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**ROS 2 Trajectory Optimization for Automated Driving based on an Optimal Control Problem (OCP).**

<img src="./assets/logo.svg" align="right" height="120" alt="trajectory_optimization logo">
<p align="justify">
This repository provides a ROS 2 node for periodically solving a nonlinear optimal control problem to generate optimized trajectories for automated driving. The goal of the OCP is to follow a <i>reference trajectory</i> while respecting the dynamics of a given vehicle model and optimizing with respect to a configured cost function, all while not violating defined constraints such as collision avoidance and road boundaries. This results in a <i>drivable trajectory</i> that fulfils all these requirements.
</p>

The open-source framework [acados](https://github.com/acados/acados) is used to define the OCP and generate the libraries for solving it online in the ROS 2 node. Key features:
- **Vehicle models**: single-track model with ackermann steering and optionally rear-wheel steering.
- **Cost function**: minimizing tracking error to reference trajectory, acceleration, jerk, and steering actuation with configurable weights.
- **Constraints**: dynamic obstacles, route boundaries, state/control/dynamics constraints.

The ROS 2 node uses the open-source ROS 2 message definitions [perception_interfaces](https://github.com/ika-rwth-aachen/perception_interfaces) and [planning_interfaces](https://github.com/ika-rwth-aachen/planning_interfaces) for all inputs and outputs, making it easy to integrate into a larger ROS 2-based system. The node is designed to be flexible and configurable, with support for different driving modes, model variants, and execution modes.

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>


> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://openads-project.github.io/), the *Open Automated Driving Systems* project. *OpenADS* and its modules have been initiated and are currently being maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/de/).


## 🚀 Quick Start

Run the ready-made demo setup from [`demo`](./demo), which starts the trajectory optimization together with dummy input generation, RViz and RQt. You can use the dynamic reconfigure options in RQt to change the parameters of the running nodes and see the effect on the generated trajectories in RViz.

1. Launch a container of the pre-built runtime image in the provided demo [Docker Compose](demo/docker-compose.yml) setup.
    ```bash
    cd demo
    xhost +local: # allow GUI forwarding from containers
    docker compose up
    ```
1. You should now see the trajectory optimization node running in the terminal, with:
    - **RViz** displaying the reference and drivable trajectories, as well as the ego vehicle and dynamic objects.
    - **RQt** showing dynamic reconfigure options. Use these to change the dummy input data for the trajectory optimization to affect the generated trajectories.
1. Stop the demo and clean up.
    > *Ctrl+C*
    ```bash
    docker compose down
    xhost -local: # revoke GUI forwarding permissions
    ```

## 💻 Development

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

Package and node interfaces are documented in the respective package READMEs listed below. Implementation details are found in the [Source Code Documentation](https://openads-project.github.io/trajectory_optimization).

| Package | Description |
| --- | --- |
| [dummy_input_generation](dummy_input_generation/README.md) | Generates and publishes dummy input data for testing purposes of the trajectory optimization node. |
| [trajectory_optimization](trajectory_optimization/README.md) | Periodically solves a nonlinear OCP to generate optimized trajectories for automated driving. |
| [trajectory_optimization_ocp](trajectory_optimization_ocp/README.md) | Defines the OCP for trajectory optimization and generates the corresponding C code headers/libraries, which are then used by `trajectory_optimization`. |

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

Development and maintenance of this repository are supported by the following projects. We acknowledge the funding of the respective institutions.

| Project | Funding Institution | Grant Number |
| --- | --- | --- |
| [AIGGREGATE](https://aiggregate.eu/) | 🇪🇺 European Union | 101202457 |
| [AIthena](https://aithena.eu/) | 🇪🇺 European Union | 101076754 |
| [autotech.agil](https://www.autotechagil.de/) | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 01IS22088A |

<p>
  <img src="https://www.drought.uni-freiburg.de/stressres/images/bmftr-logo/image" height=70>
  <img src="https://ec.europa.eu/regional_policy/images/information-sources/logo-download-center/eu_funded_en.jpg" height=70>
</p>

<sup><sub>Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor CINEA can be held responsible for them.</sup></sup>
