# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

from acados_template import AcadosOcp, AcadosOcpSolver, builders
import argparse
import os
import sys
import yaml

CURRENT_DIR_PATH = os.path.dirname(__file__)
sys.path.append(os.path.join(CURRENT_DIR_PATH, "models"))
import dims, constraints, costs, opts
import model_Ackermann, model_RWS

CURRENT_DIR_PATH = os.path.dirname(__file__)

def parseArguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', help="configuration file (.yml)", type=str, required=False, default='params.yml')
    args = parser.parse_args()
    return args

def readConfig(config):
    with open(os.path.join(CURRENT_DIR_PATH, config)) as configFile:
        params = yaml.load(configFile, yaml.FullLoader)
    return params

def main():
    args = parseArguments()
    parameters = readConfig(args.config)

    ocp = AcadosOcp()
    if parameters['model_type'] == 'Ackermann':
        model_Ackermann.set_model(ocp, parameters)
    elif parameters['model_type'] == 'RWS':
        model_RWS.set_model(ocp, parameters)
    else:
        raise ValueError(f"Unknown model type. Choose between 'Ackermann' or 'RWS'.")
    costs.set_costs(ocp, parameters)
    constraints.set_constraints(ocp, parameters) # Set constraints AFTER costs as soft constraints need to modify cost
    dims.set_dims(ocp, parameters)
    opts.set_opts(ocp, parameters)
    ocp.code_export_directory = os.path.join(CURRENT_DIR_PATH, 'c_generated_code')

    builder = builders.CMakeBuilder()
    builder.options_on = ['BUILD_ACADOS_SOLVER_LIB', 'BUILD_ACADOS_OCP_SOLVER_LIB']

    acados_tp_ocp = AcadosOcpSolver(ocp, json_file = f"{parameters['model_name']}.json", simulink_opts=None, build=True, generate=True, cmake_builder=builder)

if __name__ == '__main__':
    main()
