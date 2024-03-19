from acados_template import AcadosOcpConstraints
import numpy as np

def export_constraints() -> AcadosOcpConstraints:
    
    cons = AcadosOcpConstraints()

    # set constraints
    Fmax = 80
    cons.lbu = np.array([-Fmax])
    cons.ubu = np.array([+Fmax])
    cons.idxbu = np.array([0])

    cons.x0 = np.array([0.0, np.pi, 0.0, 0.0])

    return cons