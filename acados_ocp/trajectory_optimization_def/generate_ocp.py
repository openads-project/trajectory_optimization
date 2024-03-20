from acados_template import AcadosOcp, AcadosOcpSolver, builders
import model, dims, constraints, costs, opts
import os
import yaml
import argparse

def parser_init():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', help="configuaration file *.yml", type=str, required=False, default='params.yml')
    return parser

def readConfig(config):
    currentDirPath = os.path.dirname(os.path.realpath(__file__))
    with open(os.path.join(currentDirPath, config)) as configFile:
        config_params = yaml.load(configFile, yaml.FullLoader)
    return config_params

def main():
    parser = parser_init()
    args = parser.parse_args()
    parameters = readConfig(args.config)

    ocp = AcadosOcp()
    ocp.model = model.export_model(parameters)
    ocp.dims = dims.export_dims(ocp.model, parameters)
    ocp.constraints = constraints.export_constraints(parameters)
    ocp.cost = costs.export_costs()
    ocp.solver_options = opts.export_opts(parameters)
    ocp.code_export_directory = os.path.join(os.path.dirname(__file__), 'c_generated_code')

    builder = builders.CMakeBuilder()
    builder.options_on = ['BUILD_ACADOS_SOLVER_LIB', 'BUILD_ACADOS_OCP_SOLVER_LIB', 'BUILD_EXAMPLE', 'BUILD_SIM_EXAMPLE', 'BUILD_ACADOS_SIM_SOLVER_LIB']
    
    acados_tp_ocp = AcadosOcpSolver(ocp, json_file = 'acados_tp_ocp.json', simulink_opts=None, build=True, generate=True, cmake_builder=builder)

if __name__ == '__main__':
    main()