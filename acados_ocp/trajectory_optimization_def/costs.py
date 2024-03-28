import numpy as np
from acados_template import AcadosOcpCost
import casadi as ca


STATE_INDEX_X = 0
STATE_INDEX_Y = 1
STATE_INDEX_V = 3

P_REF_PATH_INDEX_X = 1
P_REF_PATH_INDEX_Y = 2
P_REF_PATH_INDEX_V = 3


def set_costs(ocp, config):

    # set up as external cost function
    cost = AcadosOcpCost()
    cost.cost_type = "EXTERNAL"
    cost.cost_type_e = "EXTERNAL"
    ocp.cost = cost

    # initialize parameters
    n_params_cost_weights = np.prod(config["p_cost_weights_shape"])
    n_params_ref_path = np.prod(config["p_ref_path_shape"])
    n_params = n_params_cost_weights + n_params_ref_path
    ocp.parameter_values = np.zeros(n_params)

    # get parameters
    p_cost_weights = ocp.model.p[0:n_params_cost_weights]
    p_ref_path = ocp.model.p[n_params_cost_weights:n_params_cost_weights + n_params_ref_path]

    # ca.find reference point (min distance) on reference path
    ref_path_state_dim = config["p_ref_path_shape"][1]
    # p_ref_path shuold be sortet like this: (t1, x1, y1, v1, t2, x2, y2, v2, ...)
    x_ref_path = p_ref_path[P_REF_PATH_INDEX_X::ref_path_state_dim] # every 4th element starting from index 1
    y_ref_path = p_ref_path[P_REF_PATH_INDEX_Y::ref_path_state_dim] # every 4th element starting from index 2
    v_ref_path = p_ref_path[P_REF_PATH_INDEX_V::ref_path_state_dim] # every 4th element starting from index 3
    
    dx = ca.power(x_ref_path[:] - ocp.model.x[STATE_INDEX_X], 2)
    dy = ca.power(y_ref_path[:] - ocp.model.x[STATE_INDEX_Y], 2)
    dd = ca.sqrt(dx + dy)
    idx_min = ca.find(ca.if_else(ca.mmin(dd) == dd[:], 1, 0))
    x_ref = x_ref_path[idx_min]
    y_ref = y_ref_path[idx_min]
    v_ref = v_ref_path[idx_min]

    # cost term weights
    w_x = p_cost_weights[0]
    w_y = p_cost_weights[1]
    w_v = p_cost_weights[2]

    # individual cost terms
    x_term = ca.power(ocp.model.x[0] - x_ref, 2)
    y_term = ca.power(ocp.model.x[1] - y_ref, 2)
    v_term = ca.power(ocp.model.x[3] - v_ref, 2)

    # cost functions
    ocp.model.cost_expr_ext_cost = w_x * x_term + w_y * y_term + w_v * v_term
    ocp.model.cost_expr_ext_cost_0 = w_x * x_term + w_y * y_term + w_v * v_term
    ocp.model.cost_expr_ext_cost_e = w_x * x_term + w_y * y_term + w_v * v_term
