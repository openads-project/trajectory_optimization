from acados_template import AcadosOcpConstraints
import numpy as np

def set_constraints(ocp, parameters) -> AcadosOcpConstraints:
    
    cons = AcadosOcpConstraints()

    # set constraints on state
    # set 0 < v < 30 [m/s]
    # set -a_long_max < a_lon < a_long_max [m/s^2]
    # set -delta_max < delta < delta_max [rad]
    cons.lbx = np.array([parameters['v_min'], -parameters['acceleration_lon_max'], -parameters['delta_max']])
    cons.ubx = np.array([parameters['v_max'], parameters['acceleration_lon_max'], parameters['delta_max']])
    cons.idxbx = np.array([3, 4, 6])

    # set boundaries for acceleration values through nonlinear constraints
    a_max = parameters['acceleration_max']
    cons.lh = np.array([0])
    cons.lh_e = np.array([0])
    cons.uh = np.array([a_max**2])
    cons.uh_e = np.array([a_max**2])

    # set constraints on controls
    alpha = parameters['alpha_max']
    j_lon = parameters['jerk_max']
    cons.lbu = np.array([-j_lon, -alpha])
    cons.ubu = np.array([+j_lon, +alpha])
    cons.idxbu = np.array([0, 1])

    ocp.constraints = cons