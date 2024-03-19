from acados_template import AcadosOcp, AcadosOcpSolver, builders
import model, dims, constraints, costs, opts
import os

def main():

    ocp = AcadosOcp()
    ocp.model = model.export_model()
    ocp.dims = dims.export_dims(ocp.model)
    ocp.constraints = constraints.export_constraints()
    ocp.cost = costs.export_costs()
    ocp.solver_options = opts.export_opts()
    ocp.code_export_directory = os.path.join(os.path.dirname(__file__), 'c_generated_code')

    builder = builders.CMakeBuilder()
    builder.options_on = ['BUILD_ACADOS_SOLVER_LIB', 'BUILD_ACADOS_OCP_SOLVER_LIB', 'BUILD_EXAMPLE', 'BUILD_SIM_EXAMPLE', 'BUILD_ACADOS_SIM_SOLVER_LIB']
    
    acados_tp_ocp = AcadosOcpSolver(ocp, json_file = 'acados_tp_ocp.json', simulink_opts=None, build=True, generate=True, cmake_builder=builder)

if __name__ == '__main__':
    main()