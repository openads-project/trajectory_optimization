from acados_template import AcadosOcpConstraints
import numpy as np

def export_constraints(parameters) -> AcadosOcpConstraints:
    
    cons = AcadosOcpConstraints()

    # set constraints
    alpha = parameters['alpha_max']
    j_lon = parameters['jerk_max']
    cons.lbu = np.array([-j_lon, -alpha])
    cons.ubu = np.array([+j_lon, +alpha])

    cons.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    return cons