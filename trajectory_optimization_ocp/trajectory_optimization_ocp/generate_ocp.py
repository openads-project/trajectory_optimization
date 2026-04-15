# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import argparse
import importlib
import os
import sys

import yaml
from acados_template import AcadosOcp, AcadosOcpSolver, builders

CURRENT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR_PATH = os.path.join(CURRENT_DIR_PATH, "models")
for import_path in (CURRENT_DIR_PATH, MODELS_DIR_PATH):
    if import_path not in sys.path:
        sys.path.insert(0, import_path)

constraints = importlib.import_module("constraints")
costs = importlib.import_module("costs")
dims = importlib.import_module("dims")
model_Ackermann = importlib.import_module("model_Ackermann")
model_RWS = importlib.import_module("model_RWS")
opts = importlib.import_module("opts")


def parseArguments() -> argparse.Namespace:
    """Parse command line arguments.

    Returns:
        argparse.Namespace: Parsed command line arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", help="configuration file (.yml)", type=str, required=False, default="params.yml")
    args = parser.parse_args()
    return args


def readConfig(config):
    """Read configuration from a YAML file.

    Args:
        config: Path to the configuration file.

    Returns:
        dict: Parsed configuration parameters.
    """
    with open(os.path.join(CURRENT_DIR_PATH, config)) as configFile:
        params = yaml.load(configFile, yaml.FullLoader)
    return params


def main():
    """Generate and build the optimal control problem (OCP) based on configuration."""
    args = parseArguments()
    parameters = readConfig(args.config)

    ocp = AcadosOcp()
    if parameters["model_type"] == "Ackermann":
        model_Ackermann.set_model(ocp, parameters)
    elif parameters["model_type"] == "RWS":
        model_RWS.set_model(ocp, parameters)
    else:
        raise ValueError("Unknown model type. Choose between 'Ackermann' or 'RWS'.")
    costs.set_costs(ocp, parameters)
    constraints.set_constraints(ocp, parameters)  # Set constraints AFTER costs as soft constraints need to modify cost
    dims.set_dims(ocp, parameters)
    opts.set_opts(ocp, parameters)
    ocp.code_export_directory = os.path.join(CURRENT_DIR_PATH, "c_generated_code")

    builder = builders.CMakeBuilder()
    builder.options_on = ["BUILD_ACADOS_SOLVER_LIB", "BUILD_ACADOS_OCP_SOLVER_LIB"]

    _ = AcadosOcpSolver(
        ocp, json_file=f"{parameters['model_name']}.json", simulink_opts=None, build=True, generate=True, cmake_builder=builder
    )


if __name__ == "__main__":
    main()
