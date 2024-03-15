from acados_template import AcadosOcpConstraints
import numpy as np

def export_constraints() -> AcadosOcpConstraints:
    
    cons = AcadosOcpConstraints()

    # set constraints
    alpha = 0.5
    j_lon = 4.0
    cons.lbu = np.array([-j_lon, -alpha])
    cons.ubu = np.array([+j_lon, +alpha])

    cons.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    return cons