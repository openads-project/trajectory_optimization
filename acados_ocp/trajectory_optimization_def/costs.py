import numpy as np
from acados_template import AcadosOcpCost
import casadi as ca


STATE_INDEX_X = 0
STATE_INDEX_Y = 1
STATE_INDEX_V = 3

P_REF_PATH_INDEX_X = 1
P_REF_PATH_INDEX_Y = 2
P_REF_PATH_INDEX_V = 3


def set_costs(ocp, parameter):

    # set up as external cost function
    cost = AcadosOcpCost()
    cost.cost_type = "EXTERNAL"
    cost.cost_type_e = "EXTERNAL"
    ocp.cost = cost

    # initialize parameters
    ref_init = np.zeros((parameter["nrefsamples"] * 4))
    ocp.parameter_values = ref_init

    # get parameters
    p_ref_path = ocp.model.p

    # ca.find reference point (min distance) on reference path
    dx = ca.power(p_ref_path[:, P_REF_PATH_INDEX_X] - ocp.model.x[STATE_INDEX_X], 2)
    dy = ca.power(p_ref_path[:, P_REF_PATH_INDEX_Y] - ocp.model.x[STATE_INDEX_Y], 2)
    dd = ca.sqrt(dx + dy)
    idx_min = ca.find(ca.if_else(ca.mmin(dd) == dd[:], 1, 0))
    x_ref = p_ref_path[idx_min, 1]
    y_ref = p_ref_path[idx_min, 2]
    v_ref = p_ref_path[idx_min, 3]

    # cost term weights
    w_x = 1.0
    w_y = 1.0
    w_v = 1.0

    # individual cost terms
    x_term = ca.power(ocp.model.x[0] - x_ref, 2)
    y_term = ca.power(ocp.model.x[1] - y_ref, 2)
    v_term = ca.power(ocp.model.x[3] - v_ref, 2)

    # cost functions
    ocp.model.cost_expr_ext_cost = w_x * x_term + w_y * y_term + w_v * v_term
    ocp.model.cost_expr_ext_cost_0 = w_x * x_term + w_y * y_term + w_v * v_term
    ocp.model.cost_expr_ext_cost_e = w_x * x_term + w_y * y_term + w_v * v_term
