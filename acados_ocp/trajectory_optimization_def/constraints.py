from acados_template import AcadosOcpConstraints
import numpy as np

def export_constraints(parameters) -> AcadosOcpConstraints:
    
    cons = AcadosOcpConstraints()

    # set constraints on state
    # set 0 < v < 30 [m/s]
    # set -delta_max < delta < delta_max [rad]
    cons.lbx = np.array([parameters['v_min'], -parameters['delta_max']])
    cons.ubx = np.array([parameters['v_max'], parameters['delta_max']])
    cons.idxbx = np.array([3, 6])

    # To-Do: we need to constraint the magnitude of a_lon and a_lat to a_max 
    # this is tricky since this constraint is dependant of three state variables: a_lat = v * psi and a_lon
    # a_lat * a_lat + a_lon * a_lon <= a_max * a_max

    # set constraints on controls
    alpha = parameters['alpha_max']
    j_lon = parameters['jerk_max']
    cons.lbu = np.array([-j_lon, -alpha])
    cons.ubu = np.array([+j_lon, +alpha])
    cons.idxbu = np.array([0, 1])

    return cons