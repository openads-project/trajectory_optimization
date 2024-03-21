from acados_template import AcadosOcp, AcadosOcpSolver, builders
import argparse
import os
import yaml

import model, dims, constraints, costs, opts

CURRENT_DIR_PATH = os.path.dirname(__file__)

def parseArguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', help="configuaration file (.yml)", type=str, required=False, default='params.yml')
    args = parser.parse_args()
    return args

def readConfig(config):
    with open(os.path.join(CURRENT_DIR_PATH, config)) as configFile:
        config_params = yaml.load(configFile, yaml.FullLoader)
    return config_params

def main():
    args = parseArguments()
    parameters = readConfig(args.config)

    ocp = AcadosOcp()
    model.set_model(ocp, parameters)
    dims.set_dims(ocp, parameters)
    constraints.set_constraints(ocp, parameters)
    costs.set_costs(ocp, parameters)
    opts.set_opts(ocp, parameters)
    ocp.code_export_directory = os.path.join(CURRENT_DIR_PATH, 'c_generated_code')

    builder = builders.CMakeBuilder()
    builder.options_on = ['BUILD_ACADOS_SOLVER_LIB', 'BUILD_ACADOS_OCP_SOLVER_LIB', 'BUILD_EXAMPLE', 'BUILD_SIM_EXAMPLE', 'BUILD_ACADOS_SIM_SOLVER_LIB']
    
    acados_tp_ocp = AcadosOcpSolver(ocp, json_file = 'acados_tp_ocp.json', simulink_opts=None, build=True, generate=True, cmake_builder=builder)

if __name__ == '__main__':
    main()