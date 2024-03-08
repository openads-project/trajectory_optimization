from acados_template import AcadosOcp, AcadosOcpSolver
from model import export_vehicle_model
from opts import export_opts
from dims import export_dims

def main():

    ocp = AcadosOcp()
    ocp.model = export_vehicle_model()
    ocp.dims = export_dims(ocp.model)
    ocp.solver_options = export_opts()

    ocp_solver = AcadosOcpSolver(ocp, json_file = 'ocp.json', simulink_opts=None, build=False, generate=True)

if __name__ == '__main__':
    main()