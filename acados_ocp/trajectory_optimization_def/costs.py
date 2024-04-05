import numpy as np
from acados_template import AcadosOcpCost
from constants import *
import casadi as ca


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
    n_params_max_vel = np.prod(config["p_max_vel_shape"])
    n_params_s_ref = np.prod(config["p_s_ref_shape"])
    n_obstacles = np.prod(config["p_obstacles_shape"])
    n_params = n_params_cost_weights + n_params_ref_path + n_params_max_vel + n_params_s_ref + n_obstacles
    ocp.parameter_values = np.zeros(n_params)

    # get parameters
    p_cost_weights = ocp.model.p[0:n_params_cost_weights]
    p_ref_path = ocp.model.p[n_params_cost_weights:n_params_cost_weights + n_params_ref_path]
    p_max_vel = ocp.model.p[n_params_cost_weights + n_params_ref_path:n_params_cost_weights + n_params_ref_path + n_params_max_vel]
    p_s_ref = ocp.model.p[n_params_cost_weights + n_params_ref_path + n_params_max_vel:n_params_cost_weights + n_params_ref_path + n_params_max_vel + n_params_s_ref]
    p_obstacles = ocp.model.p[n_params_cost_weights + n_params_ref_path + n_params_max_vel + n_params_s_ref:n_params]

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

    # Find nearest adjacent sample on reference path
    condition_begin = (idx_min == 0)
    condition_end = (idx_min == p_ref_path.rows()-1)
    condition_intermediate = ca.logic_and(ca.logic_not(condition_begin), ca.logic_not(condition_end))
    dist_1 = ca.if_else(condition_intermediate, dd[idx_min-1], ca.MX_inf(1,1), True)
    dist_2 = ca.if_else(condition_intermediate, dd[idx_min+1], ca.MX_inf(1,1), True)
    condition_dist = (dist_1 < dist_2)
    next_idx_min = ca.if_else(condition_begin, idx_min+1, ca.if_else(condition_end, idx_min-1, ca.if_else(condition_dist, idx_min-1, idx_min+1, True), True), True)

    # We now want to compute the shortest distance between the state-point and a line segment idx_min---next_idx_min
    # Extend the segment to a complete line first; determine point with shortest distance to state-point (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line), but formulate as parameter lambda
    # Values [0, 1] for lambda mean the nearest point is on the segment and the computed distance is perpendicular to the line segment
    # Note that lambda must be >=0 due to the way we defined the line segment
    x1 = x_ref_path[idx_min]
    y1 = y_ref_path[idx_min]
    v1 = v_ref_path[idx_min]
    x2 = x_ref_path[next_idx_min]
    y2 = y_ref_path[next_idx_min]
    v2 = v_ref_path[next_idx_min]

    c_square = ca.power(x2-x1,2)+ca.power(y2-y1,2)
    lmd = ((ocp.model.x[STATE_INDEX_X] - x1)*(x2-x1) + (ocp.model.x[STATE_INDEX_Y] - y1)*(y2-y1))/ c_square
    x_ref_inter = x1 + lmd * (x2-x1)
    y_ref_inter = y1 + lmd * (y2-y1)
    v_ref_inter = v1 + lmd * (v2-v1)
    dlon = lmd * ca.sqrt(c_square)
    dlat = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X]-x_ref_inter,2)+ca.power(ocp.model.x[STATE_INDEX_Y]-y_ref_inter,2))

    # obstacles
    r_ego = p_obstacles[2] # TODO: param for r_ego?
    d_obstacles_min = 0.5 # TODO: param for d_obstacles_min?
    # dist_obstacles = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X] - p_obstacles[0], 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - p_obstacles[1], 2))
    dist_obstacles = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X] - 5, 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - 0, 2))
    # conditional_obstacles_term = ca.if_else(dist_obstacles <= r_ego + p_obstacles[2] + d_obstacles_min, dist_obstacles - (r_ego + p_obstacles[2] + d_obstacles_min), 0)
    conditional_obstacles_term = ca.if_else(dist_obstacles <= r_ego + 0.5 + d_obstacles_min, dist_obstacles, 0)
    obstacles_term = ca.power(conditional_obstacles_term, 2)

    # cost term weights
    w_lon = p_cost_weights[0]
    w_lat = p_cost_weights[1]
    w_x = p_cost_weights[2]
    w_y = p_cost_weights[3]
    w_v = p_cost_weights[4]
    w_v_max = p_cost_weights[5]
    w_s_ref = p_cost_weights[6]
    w_obstacles = p_cost_weights[7]

    # individual cost terms
    dlon_term = ca.power(dlon, 2)
    dlat_term = ca.power(dlat, 2)
    x_term = ca.power(ocp.model.x[STATE_INDEX_X] - x_ref, 2)
    y_term = ca.power(ocp.model.x[STATE_INDEX_Y] - y_ref, 2)
    v_term = ca.power(ocp.model.x[STATE_INDEX_V] - v_ref, 2)
    v_max_term = ca.power(ocp.model.x[STATE_INDEX_V] - p_max_vel, 2)
    s_ref_term = ca.power(ocp.model.x[STATE_INDEX_S] - p_s_ref, 2)

    # cost functions
    ocp.model.cost_expr_ext_cost = w_lon * dlon_term + w_lat * dlat_term + w_x * x_term + w_y * y_term + w_v * v_term + w_v_max * v_max_term + w_obstacles * obstacles_term
    ocp.model.cost_expr_ext_cost_0 = w_lon * dlon_term + w_lat * dlat_term +  w_x * x_term + w_y * y_term + w_v * v_term + w_v_max * v_max_term + w_obstacles * obstacles_term
    ocp.model.cost_expr_ext_cost_e = w_lon * dlon_term + w_lat * dlat_term + w_x * x_term + w_y * y_term + w_v * v_term + w_v_max * v_max_term + w_s_ref * s_ref_term + w_obstacles * obstacles_term
