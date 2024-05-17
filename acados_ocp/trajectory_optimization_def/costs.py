import numpy as np
from acados_template import AcadosOcpCost, AcadosOcp
from constants import *
import casadi as ca

def set_costs(ocp: AcadosOcp, config):

    # set up as external cost function
    cost = AcadosOcpCost()
    cost.cost_type = "EXTERNAL"
    cost.cost_type_e = "EXTERNAL"
    ocp.cost = cost

    # initialize parameters
    n_params_cost_weights = np.prod(config["p_cost_weights_shape"])
    n_params_dynamic_weight = 1
    n_params_ref_path = np.prod(config["p_ref_path_shape"])
    n_params_v_max = 1
    n_params_s_ref = 1
    n_obstacles = np.prod(config["p_obstacles_shape"])
    n_params = n_params_cost_weights + n_params_dynamic_weight + n_params_ref_path + n_params_v_max + n_params_s_ref + n_obstacles
    ocp.parameter_values = np.zeros(n_params)

    # get parameters
    idx_params = 0
    p_cost_weights = ocp.model.p[idx_params:(idx_params := idx_params + n_params_cost_weights)]
    p_dynamic_weight = ocp.model.p[idx_params:(idx_params := idx_params + n_params_dynamic_weight)]
    p_ref_path = ocp.model.p[idx_params:(idx_params := idx_params + n_params_ref_path)]
    p_v_max = ocp.model.p[idx_params:(idx_params := idx_params + n_params_v_max)]
    p_s_ref = ocp.model.p[idx_params:(idx_params := idx_params + n_params_s_ref)]
    p_obstacles = ocp.model.p[idx_params:(idx_params := idx_params + n_obstacles)]
    assert idx_params == n_params

    # cost term weights
    w_lon = p_cost_weights[0]
    w_lat = p_cost_weights[1]
    w_x = p_cost_weights[2]
    w_y = p_cost_weights[3]
    w_v = p_cost_weights[4]
    w_v_max = p_cost_weights[5]
    w_s_ref = p_cost_weights[6]
    w_obstacles = p_cost_weights[7]
    w_a = p_cost_weights[8]
    w_j_lat = p_cost_weights[9]
    w_j_lon = p_cost_weights[10]
    w_alpha = p_cost_weights[11]
    w_end_yaw = p_cost_weights[12]

    # define running-costs
    # reference path costs
    ref_path_costs = calc_ref_path_cost(ocp, config, p_ref_path)
    ocp.model.cost_expr_ext_cost = p_dynamic_weight * w_lon * ref_path_costs["dlon"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_lat * ref_path_costs["dlat"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_v * ref_path_costs["v"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_x * ref_path_costs["x"] 
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_y * ref_path_costs["y"]
    # control variable costs
    input_costs = calc_control_cost(ocp, config)
    ocp.model.cost_expr_ext_cost += w_j_lon * input_costs["j_lon"]
    ocp.model.cost_expr_ext_cost += w_alpha * input_costs["alpha"]
    # acceleration magnitude costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_a * calc_a_cost(ocp, config)
    # lateral jerk costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_j_lat * calc_j_lat_cost(ocp, config)
    # obstacle costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_obstacles * calc_obstacles_cost(ocp, config, p_obstacles)
    # v-max costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_v_max * calc_v_max_cost(ocp, config, p_v_max)

    # define terminal-costs
    # end yaw
    ocp.model.cost_expr_ext_cost_e = w_end_yaw * ref_path_costs["dpsi"]
    # end v
    ocp.model.cost_expr_ext_cost_e += p_dynamic_weight * w_v * ref_path_costs["v"]
    # final distance
    ocp.model.cost_expr_ext_cost_e += w_s_ref * calc_s_max_cost(ocp, config, p_s_ref) 

    # define inital-costs
    ocp.model.cost_expr_ext_cost_0 = ocp.model.cost_expr_ext_cost

def calc_ref_path_cost(ocp: AcadosOcp, config: dict, p_ref_path: ca.MX) -> dict:
    n_params_ref_path = np.prod(config["p_ref_path_shape"])

    # consider only the actual reference path (could be smaller than the parameter space; identify by first infinite value)
    idx_inf = n_params_ref_path
    for i in range(n_params_ref_path):
        if p_ref_path[i] == ca.MX_inf:
            idx_inf = i
            break
    p_ref_path = p_ref_path[:idx_inf]

    # ca.find reference point (min distance) on reference path
    ref_path_state_dim = config["p_ref_path_shape"][1]
    # p_ref_path shuold be sortet like this: (psi1, x1, y1, v1, psi2, x2, y2, v2, ...)
    psi_ref_path = p_ref_path[P_REF_PATH_INDEX_PSI::ref_path_state_dim] # every 4th element starting from index 0
    x_ref_path = p_ref_path[P_REF_PATH_INDEX_X::ref_path_state_dim] # every 4th element starting from index 1
    y_ref_path = p_ref_path[P_REF_PATH_INDEX_Y::ref_path_state_dim] # every 4th element starting from index 2
    v_ref_path = p_ref_path[P_REF_PATH_INDEX_V::ref_path_state_dim] # every 4th element starting from index 3

    dx = ca.power(x_ref_path[:] - ocp.model.x[STATE_INDEX_X], 2)
    dy = ca.power(y_ref_path[:] - ocp.model.x[STATE_INDEX_Y], 2)
    dd = ca.sqrt(dx + dy)
    idx_min = ca.find(ca.if_else(ca.mmin(dd) == dd[:], 1, 0))
    x_ref = x_ref_path[idx_min]
    y_ref = y_ref_path[idx_min]

    # Find nearest adjacent sample on reference path
    condition_begin = (idx_min == 0)
    condition_end = (idx_min == x_ref_path.rows()-1)
    condition_intermediate = ca.logic_and(ca.logic_not(condition_begin), ca.logic_not(condition_end))
    dist_1 = ca.if_else(condition_intermediate, dd[idx_min-1], ca.MX_inf(1,1), True)
    dist_2 = ca.if_else(condition_intermediate, dd[idx_min+1], ca.MX_inf(1,1), True)
    condition_dist = (dist_1 < dist_2)
    next_idx_min = ca.if_else(condition_begin, idx_min+1, ca.if_else(condition_end, idx_min-1, ca.if_else(condition_dist, idx_min-1, idx_min+1, True), True), True)

    # We now want to compute the shortest distance between the state-point and a line segment idx_min---next_idx_min
    # Extend the segment to a complete line first; determine point with shortest distance to state-point (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line), but formulate as parameter lambda
    # Values [0, 1] for lambda mean the nearest point is on the segment and the computed distance is perpendicular to the line segment
    # Note that lambda must be >=0 due to the way we defined the line segment
    psi1 = psi_ref_path[idx_min]
    x1 = x_ref_path[idx_min]
    y1 = y_ref_path[idx_min]
    v1 = v_ref_path[idx_min]
    psi2 = psi_ref_path[next_idx_min]
    x2 = x_ref_path[next_idx_min]
    y2 = y_ref_path[next_idx_min]
    v2 = v_ref_path[next_idx_min]

    dxy_sq = ca.power(x2 - x1, 2) + ca.power(y2 - y1, 2)
    lmd = ca.if_else((dxy_sq == 0), 0, ((ocp.model.x[STATE_INDEX_X] - x1) * (x2 - x1) + (ocp.model.x[STATE_INDEX_Y] - y1) * (y2 - y1)) / dxy_sq)
    # allow extrapolation for beginning of reference but not at the end (to penalize overshooting)
    lmd = ca.if_else(condition_begin, lmd, ca.fmin(ca.fmax(lmd, 0), 1))
    x_ref_inter = x1 + lmd * (x2 - x1)
    y_ref_inter = y1 + lmd * (y2 - y1)
    lmd = ca.fmin(ca.fmax(lmd, 0), 1)
    psi_ref_inter = psi1 + lmd * (psi2 - psi1)
    v_ref_inter = v1 + lmd * (v2 - v1)
    dlon = lmd * (ca.sqrt(dxy_sq), 2)
    dlat = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X]-x_ref_inter, 2)+ca.power(ocp.model.x[STATE_INDEX_Y]-y_ref_inter, 2))

    # longitudinal deviation term
    dlon_term = ca.power(dlon / config["c_lon"], 2)
    # lateral deviation term
    dlat_term = ca.power(dlat / config["c_lat"], 2)
    # x deviation term
    x_term = ca.power((ocp.model.x[STATE_INDEX_X] - x_ref) / config["c_x"], 2)
    # y deviation term
    y_term = ca.power((ocp.model.x[STATE_INDEX_Y] - y_ref) / config["c_y"], 2)
    # v deviation term
    # first ensure that the reference velocity is > 0
    v_ref = ca.fmax(v_ref_inter, 0.0)
    # we define the scaling value of v to v_ref: a velocity deviation of v_ref leads to a cost of 1
    # for numeric stability (low reference speeds) we ensure that v_scale > V_SCALE_MIN > 0
    v_scale = ca.fmax(v_ref, V_SCALE_MIN)
    v_term = ca.power((v_ref - ocp.model.x[STATE_INDEX_V]) / v_scale, 2)
    # psi deviation term
    psi_term = ca.power(ocp.model.x[STATE_INDEX_PSI] - psi_ref_inter, 2)

    cost_terms = {"dlon": dlon_term, "dlat": dlat_term, "dpsi": psi_term, "x": x_term, "y": y_term, "v": v_term}
    return cost_terms

def calc_obstacles_cost(ocp: AcadosOcp, config: dict, p_obstacles: ca.MX) -> ca.MX:
    # obstacles
    # r_ego = p_obstacles[2] # TODO: param for r_ego?
    # d_obstacles_min = 0.5 # TODO: param for d_obstacles_min?
    # dist_obstacles = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X] - p_obstacles[0], 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - p_obstacles[1], 2))
    # dist_obstacles = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X] - 5, 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - 0, 2))
    # conditional_obstacles_term = ca.if_else(dist_obstacles <= r_ego + p_obstacles[2] + d_obstacles_min, dist_obstacles - (r_ego + p_obstacles[2] + d_obstacles_min), 0)
    # conditional_obstacles_term = ca.if_else(dist_obstacles <= r_ego + 0.5 + d_obstacles_min, dist_obstacles, 0)
    # obstacles_term = ca.power(conditional_obstacles_term, 2)

    MIN_D_LONG = 1.0
    MIN_D_LAT = 1.0
    obj_length = 1.0
    obj_width = 1.0
    ego_length = 1.0
    ego_width = 1.0
    # dT = v_ego * tau
    dLongMin = MIN_D_LONG + 0.5 * obj_length + 0.5 * ego_length # + dT ; positions in geometric center
    dLatMin = MIN_D_LAT + 0.5 * obj_width + 0.5 * ego_width

    # TODO: handle more objects
    dLong = ca.fabs(ocp.model.x[STATE_INDEX_X] - p_obstacles[0])
    dLat = ca.fabs(ocp.model.x[STATE_INDEX_Y] - p_obstacles[1])

    aLat = ca.pi / dLatMin # TODO: get dLatMin from param
    cLat = ca.cos(aLat * dLat) + 1
    aLong = ca.pi / dLongMin # TODO: get dLongMin from param
    cLong = ca.cos(aLong * dLong) + 1
    cObst = cLat * cLong

    obst_condition = ca.logic_or(dLat > dLatMin, dLong > dLongMin) 
    obstacles_term = ca.if_else(obst_condition, 0, cObst)

    return obstacles_term

def calc_control_cost(ocp: AcadosOcp, config: dict) -> dict:
    j_lon_term = ca.power(ocp.model.u[CONTROL_INDEX_J_LON] / config["c_jlon"], 2)
    alpha_term = ca.power(ocp.model.u[CONTROL_INDEX_ALPHA] / config["c_alpha"], 2)

    cost_terms = {"j_lon": j_lon_term, "alpha": alpha_term}
    return cost_terms

def calc_a_cost(ocp: AcadosOcp, config: dict) -> ca.MX:
    # single track model parameters
    l = config["wheelbase"]
    v = ocp.model.x[STATE_INDEX_V]
    tan_delta = ca.fmax(-10, ca.fmin(10, ca.tan(ocp.model.x[STATE_INDEX_DELTA])))
    # derive psi_dot
    psi_dot = v / l * tan_delta
    # derive a_lat
    a_lat = v * psi_dot
    # a_lon is given as state vaiable
    a_lon = ocp.model.x[STATE_INDEX_A_LON]

    a_term = ca.power(a_lon, 2) + ca.power(a_lat, 2)/ ca.power(config["c_a"], 2)

    return a_term

def calc_j_lat_cost(ocp: AcadosOcp, config: dict) -> ca.MX:
    l = config["wheelbase"]
    v = ocp.model.x[STATE_INDEX_V]
    a_lon = ocp.model.x[STATE_INDEX_A_LON]
    tan_delta = ca.fmax(-10, ca.fmin(10, ca.tan(ocp.model.x[STATE_INDEX_DELTA])))
    alpha = ocp.model.u[CONTROL_INDEX_ALPHA]

    j_lat = 2 * v * a_lon / l * tan_delta + ca.power(v, 2) / l * alpha * (1.0 + ca.power(tan_delta, 2))
    j_lat_term = ca.power(j_lat / config["c_jlat"], 2)

    return j_lat_term

def calc_v_max_cost(ocp: AcadosOcp, config: dict, p_v_max: ca.MX) -> ca.MX:
    v_max_term = ca.power(ocp.model.x[STATE_INDEX_V] - p_v_max, 2)

    return v_max_term

def calc_s_max_cost(ocp: AcadosOcp, config: dict, p_s_ref: ca.MX) -> ca.MX:
    s_ref_term = ca.power(ocp.model.x[STATE_INDEX_S] - p_s_ref, 2)

    return s_ref_term