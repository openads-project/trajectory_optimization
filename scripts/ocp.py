from acados_template import AcadosOcp, AcadosOcpSolver
from model import export_vehicle_model

def main():

    ocp = AcadosOcp()
    ocp.model = export_vehicle_model()
    
    Tf = 5.0
    N = 50