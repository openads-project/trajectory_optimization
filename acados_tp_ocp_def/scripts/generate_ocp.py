from acados_template import AcadosOcp, AcadosOcpSolver
import model, dims, constraints, costs, opts

def main():

    ocp = AcadosOcp()
    ocp.model = model.export_model()
    ocp.dims = dims.export_dims()
    ocp.constraints = constraints.export_constraints()
    ocp.cost = costs.export_costs()  
    ocp.solver_options = opts.export_opts()

    acados_tp_ocp = AcadosOcpSolver(ocp, json_file = 'acados_tp_ocp.json', simulink_opts=None, build=False, generate=True)

if __name__ == '__main__':
    main()