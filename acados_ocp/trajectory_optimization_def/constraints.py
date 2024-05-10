from acados_template import AcadosOcpConstraints, AcadosOcp
from casadi import vertcat, fmin, fmax, tan
from constants import *
import numpy as np

def set_constraints(ocp: AcadosOcp, parameters):
    
    cons = AcadosOcpConstraints()
    
    l = parameters['wheelbase']

    # set constraints on state
    # set v_min < v < v_max [m/s]
    # set -a_lon_max < a_lon < a_lon_max [m/s^2]
    # set -delta_max < delta < delta_max [rad]
    cons.lbx = np.array([parameters['v_min'], -parameters['acceleration_lon_max'], -parameters['delta_max']])
    cons.ubx = np.array([parameters['v_max'], parameters['acceleration_lon_max'], parameters['delta_max']])
    cons.idxbx = np.array([STATE_INDEX_V, STATE_INDEX_A_LON, STATE_INDEX_DELTA])

    # set constraints on controls
    alpha = parameters['alpha_max']
    j_lon = parameters['jerk_max']
    cons.lbu = np.array([-j_lon, -alpha])
    cons.ubu = np.array([+j_lon, +alpha])
    cons.idxbu = np.array([CONTROL_INDEX_J_LON, CONTROL_INDEX_ALPHA])

    # define nonlinear constraint expression for acceleration
    # a <= sqrt(a_lon^2 + a_lat^2) i.e. a^2 <= a_lon^2 + a_lat^2
    # a_lat = v * psiDot
    psi_dot = ocp.model.x[STATE_INDEX_V] / l * fmax(-10, fmin(10, tan(ocp.model.x[STATE_INDEX_DELTA])))
    a_lat = ocp.model.x[STATE_INDEX_V] * psi_dot
    a_squared = ocp.model.x[STATE_INDEX_A_LON]**2 + a_lat**2
    ocp.model.con_h_expr = vertcat(a_squared)
    ocp.model.con_h_expr_e = vertcat(a_squared)

    # set boundaries for acceleration values through nonlinear constraints
    a_max = parameters['acceleration_max']
    cons.lh = np.array([0])
    cons.lh_e = np.array([0])
    cons.uh = np.array([a_max**2])
    cons.uh_e = np.array([a_max**2])
    
    # Add slack to state constraints
    # Here, we add a slack to velocity and acceleration constraints, but NOT to the steering angle
    # This might make the optimization problem unfeasible, but we just cannot physically soften the steering angle constraint
    cons.idxsbx = np.array([0, 1])        # Index of state bounds that are softened -> indices correspond to cons.idxbx
    cons.idxsh = np.array([0])            # Index of nonlinear constraints that are softened -> indices correspond to entries in con_h_expr
    # In the cost terms, the slack variables are arranged  as follows: idxsbu, idxsbx, idxsg, idxsh
    # So here, we have      v,    a_lon, a_squared
    ocp.cost.Zl = np.diag(parameters["slack_weights"]["quadratic_lower"])   # Quadratic cost on lower bound slack variables 
    ocp.cost.Zu = np.diag(parameters["slack_weights"]["quadratic_upper"])   # Quadratic cost on upper bound slack variables
    ocp.cost.zl = np.array(parameters["slack_weights"]["linear_lower"])     # Linear cost on lower bound slack variables
    ocp.cost.zu = np.array(parameters["slack_weights"]["linear_upper"])     # Linear cost on upper bound slack variables

    # set initial condition
    # Note that this internally is mapped to idxbx_0=range(nx), lbx_0=x0, ubx_0=x0, so when setting these in the C++ node for step 0, all variables can be constrained.
    cons.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    ocp.constraints = cons