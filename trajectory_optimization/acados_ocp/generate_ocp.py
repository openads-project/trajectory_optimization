from acados_template import AcadosOcp, AcadosOcpSolver, builders
import model, dims, constraints, costs, opts

def main():

    ocp = AcadosOcp()
    ocp.model = model.export_model()
    ocp.dims = dims.export_dims()
    ocp.constraints = constraints.export_constraints()
    ocp.cost = costs.export_costs()  
    ocp.solver_options = opts.export_opts()
    ocp.code_export_directory = 'c_generated_code'

    acados_tp_ocp = AcadosOcpSolver(ocp, json_file = 'acados_tp_ocp.json', simulink_opts=None, build=True, generate=True, cmake_builder=builders.CMakeBuilder())

if __name__ == '__main__':
    main()