from acados_template import AcadosOcpConstraints
from casadi import vertcat
from constants import *
import numpy as np

def set_constraints(ocp, parameters):
    
    cons = AcadosOcpConstraints()

    # set constraints on state
    # set 0 < v < 30 [m/s]
    # set -a_lon_max < a_lon < a_lon_max [m/s^2]
    # set -delta_max < delta < delta_max [rad]
    cons.lbx = np.array([parameters['v_min'], -parameters['acceleration_lon_max'], -parameters['delta_max']])
    cons.ubx = np.array([parameters['v_max'], parameters['acceleration_lon_max'], parameters['delta_max']])
    cons.idxbx = np.array([3, 4, 6])


    # set constraints on controls
    alpha = parameters['alpha_max']
    j_lon = parameters['jerk_max']
    cons.lbu = np.array([-j_lon, -alpha])
    cons.ubu = np.array([+j_lon, +alpha])
    cons.idxbu = np.array([0, 1])

    # define nonlinear constraint expression for acceleration
    # a <= sqrt(a_lon^2 + a_lat^2) i.e. a^2 <= a_lon^2 + a_lat^2
    # a_lat = v * psi
    # state vector index 3 is v, index 4 is a_lon, index 5 is psi
    a_lat = ocp.model.x[STATE_INDEX_V] * ocp.model.x[STATE_INDEX_PSI]
    a_squared = ocp.model.x[STATE_INDEX_A_LON]**2 + a_lat**2
    ocp.model.con_h_expr = vertcat(a_squared)
    ocp.model.con_h_expr_e = vertcat(a_squared)

    # set boundaries for acceleration values through nonlinear constraints
    a_max = parameters['acceleration_max']
    cons.lh = np.array([0])
    cons.lh_e = np.array([0])
    cons.uh = np.array([a_max**2])
    cons.uh_e = np.array([a_max**2])

    # set initial condition
    cons.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    ocp.constraints = cons