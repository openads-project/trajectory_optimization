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
    n_params_obstacles = np.prod(config["p_obstacles_shape"])
    n_params = n_params_cost_weights + n_params_dynamic_weight + n_params_ref_path + n_params_obstacles
    ocp.parameter_values = np.zeros(n_params)

    # get parameters
    idx_params = 0
    p_cost_weights = ocp.model.p[idx_params:(idx_params := idx_params + n_params_cost_weights)]
    p_dynamic_weight = ocp.model.p[idx_params:(idx_params := idx_params + n_params_dynamic_weight)]
    p_ref_path = ocp.model.p[idx_params:(idx_params := idx_params + n_params_ref_path)]
    p_obstacles = ocp.model.p[idx_params:(idx_params := idx_params + n_params_obstacles)]
    assert idx_params == n_params

    # cost term weights
    w_lat = p_cost_weights[0]
    w_x = p_cost_weights[1]
    w_y = p_cost_weights[2]
    w_v = p_cost_weights[3]
    w_obstacles = p_cost_weights[4]
    w_a = p_cost_weights[5]
    w_j_lat = p_cost_weights[6]
    w_j_lon = p_cost_weights[7]
    w_alpha = p_cost_weights[8]
    w_end_yaw = p_cost_weights[9]

    # define running-costs
    # reference path costs
    ref_path_costs = calc_ref_path_cost(ocp, config, p_ref_path)
    ocp.model.cost_expr_ext_cost = p_dynamic_weight * w_lat * ref_path_costs["dlat"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_x * ref_path_costs["x"] 
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_y * ref_path_costs["y"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_v * ref_path_costs["v"]
    # obstacle costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_obstacles * calc_obstacles_cost(ocp, config, p_obstacles)
    # acceleration magnitude costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_a * calc_a_cost(ocp, config)
    # lateral jerk costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_j_lat * calc_j_lat_cost(ocp, config)
    # control variable costs
    control_costs = calc_control_cost(ocp, config)
    ocp.model.cost_expr_ext_cost += w_j_lon * control_costs["j_lon"]
    ocp.model.cost_expr_ext_cost += w_alpha * control_costs["alpha"]

    # define terminal-costs
    # end v
    ocp.model.cost_expr_ext_cost_e = p_dynamic_weight * w_v * ref_path_costs["v"]
    # end yaw
    ocp.model.cost_expr_ext_cost_e += w_end_yaw * ref_path_costs["dpsi"]

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

    # find nearest adjacent sample on reference path
    condition_begin = (idx_min == 0)
    condition_end = (idx_min == x_ref_path.rows()-1)
    condition_intermediate = ca.logic_and(ca.logic_not(condition_begin), ca.logic_not(condition_end))
    dist_1 = ca.if_else(condition_intermediate, dd[idx_min-1], ca.MX_inf(1,1), True)
    dist_2 = ca.if_else(condition_intermediate, dd[idx_min+1], ca.MX_inf(1,1), True)
    condition_dist = (dist_1 < dist_2)
    next_idx_min = ca.if_else(condition_begin, idx_min+1, ca.if_else(condition_end, idx_min-1, ca.if_else(condition_dist, idx_min-1, idx_min+1, True), True), True)

    # compute the shortest distance between the state-point and a line segment idx_min---next_idx_min
    # extend the segment to a complete line first; determine point with shortest distance to state-point (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line), but formulate as parameter lambda
    # values [0, 1] for lambda mean the nearest point is on the segment and the computed distance is perpendicular to the line segment
    # note that lambda must be >=0 due to the way we defined the line segment
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
    dlat = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X]-x_ref_inter, 2)+ca.power(ocp.model.x[STATE_INDEX_Y]-y_ref_inter, 2))

    # lateral deviation term
    dlat_term = ca.power(dlat, 2) / ca.power(config["c_lat"], 2)
    # x deviation term
    x_term = ca.power((ocp.model.x[STATE_INDEX_X] - x_ref), 2) / ca.power(config["c_x"], 2)
    # y deviation term
    y_term = ca.power((ocp.model.x[STATE_INDEX_Y] - y_ref), 2) / ca.power(config["c_y"], 2)
    # v deviation term
    # first ensure that the reference velocity is > 0
    v_ref = ca.fmax(v_ref_inter, 0.0)
    # we define the scaling value of v to v_ref: a velocity deviation of v_ref leads to a cost of 1
    # for numeric stability (low reference speeds) we ensure that v_scale > V_SCALE_MIN > 0
    v_scale = ca.fmax(v_ref, V_SCALE_MIN)
    v_term = ca.power((v_ref - ocp.model.x[STATE_INDEX_V]), 2) / ca.power(v_scale, 2)
    # psi deviation term
    psi_term = ca.power(ocp.model.x[STATE_INDEX_PSI] - psi_ref_inter, 2)

    cost_terms = {"dlat": dlat_term, "dpsi": psi_term, "x": x_term, "y": y_term, "v": v_term}
    return cost_terms

def calc_obstacles_cost(ocp: AcadosOcp, config: dict, p_obstacles: ca.MX) -> ca.MX:
    # obstacles: LRE implementation
    # r_ego = p_obstacles[2] # TODO: param for r_ego?
    # d_obstacles_min = 0.5 # TODO: param for d_obstacles_min?
    # dist_obstacles = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X] - p_obstacles[0], 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - p_obstacles[1], 2))
    # dist_obstacles = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X] - 5, 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - 0, 2))
    # conditional_obstacles_term = ca.if_else(dist_obstacles <= r_ego + p_obstacles[2] + d_obstacles_min, dist_obstacles - (r_ego + p_obstacles[2] + d_obstacles_min), 0)
    # conditional_obstacles_term = ca.if_else(dist_obstacles <= r_ego + 0.5 + d_obstacles_min, dist_obstacles, 0)
    # obstacles_term = ca.power(conditional_obstacles_term, 2)

    # ===== NEW =====

    # consider only the relevant obstacles (could be smaller than the parameter space; identify by first infinite value)
    n_params_obstacles = np.prod(config["p_obstacles_shape"])
    idx_inf = n_params_obstacles
    for i in range(n_params_obstacles):
        if p_obstacles[i] == ca.MX_inf:
            idx_inf = i
            break
    p_obstacles = p_obstacles[:idx_inf]

    D_MIN_OBSTACLE = 0.5
    circle_approximation_radius_ego = ca.sqrt(ca.power(config["width"], 2) + ca.power(config["length"], 2)) / 2.0
    obstacles_term = 0.0
    obstacle_state_dim = config["p_obstacles_shape"][1]
    n_obstacles = p_obstacles.rows() // obstacle_state_dim
    for i in range(n_obstacles):
        x_center = p_obstacles[i * obstacle_state_dim + P_OBSTACLES_INDEX_X]
        y_center = p_obstacles[i * obstacle_state_dim + P_OBSTACLES_INDEX_Y]
        yaw = p_obstacles[i * obstacle_state_dim + P_OBSTACLES_INDEX_YAW]
        length = p_obstacles[i * obstacle_state_dim + P_OBSTACLES_INDEX_LENGTH]
        width = p_obstacles[i * obstacle_state_dim + P_OBSTACLES_INDEX_WIDTH]
        # geometry approximation of object
        n_circles = determine_n_discretization_circles(length, width)
        n_circles = 3
        circle_centers_x, circle_centers_y, r_obstacle = approximate_object_geometry(n_circles, x_center, y_center, yaw, length, width)
        # find the object circle that is the nearest to the vehicle and store dLat and dLong
        if n_circles > 1:
            dx = ca.power(circle_centers_x[:] - ocp.model.x[STATE_INDEX_X], 2)
            dy = ca.power(circle_centers_y[:] - ocp.model.x[STATE_INDEX_Y], 2)
            dd = ca.sqrt(dx + dy)
            # index min gives us the index of the circle that is nearest to the ego-vehicle-circle
            idx_min = ca.find(ca.if_else(ca.mmin(dd) == dd[:], 1, 0))
            dLong = ca.fabs(ocp.model.x[STATE_INDEX_X] - circle_centers_x[idx_min])
            dLat = ca.fabs(ocp.model.x[STATE_INDEX_Y] - circle_centers_y[idx_min])
        else:
            dLong = ca.fabs(ocp.model.x[STATE_INDEX_X] - circle_centers_x[0])
            dLat = ca.fabs(ocp.model.x[STATE_INDEX_Y] - circle_centers_x[0])

        # define minimum lateral and longitudinal distance to object circles
        dLatMin = D_MIN_OBSTACLE  + circle_approximation_radius_ego + r_obstacle
        dLongMin = D_MIN_OBSTACLE  + circle_approximation_radius_ego + r_obstacle
        # calculate cost for object-circle that shows the minimum distance to the ego-vehicle-circle
        cLong = ca.cos(ca.pi / dLongMin * dLong) + 1
        cLat = ca.cos(ca.pi / dLatMin * dLat) + 1
        cObst = cLat * cLong
        obst_condition = ca.logic_or(dLat > dLatMin, dLong > dLongMin)
        obstacles_term += ca.if_else(obst_condition, 0, cObst)

    # =================

    # currently not working and overwriting with adp implementation
    # p_obstacles shuold be sortet like this: (x1, y1, r1, x2, y2, r2, ...)
    # for i in range(n_obstacles):
    #     x_obstacle = p_obstacles[i*obstacle_state_dim + P_OBSTACLES_INDEX_X]
    #     y_obstacle = p_obstacles[i*obstacle_state_dim + P_OBSTACLES_INDEX_Y]
    #     r_obstacle = p_obstacles[i*obstacle_state_dim + P_OBSTACLES_INDEX_R]
    #     dist_obstacle = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X] - x_obstacle, 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - y_obstacle, 2))
    #     conditional_obstacle_term = ca.if_else(dist_obstacle <= r_obstacle, r_obstacle - dist_obstacle, 0)
    #     if i == 0:
    #         obstacles_term = ca.power(conditional_obstacle_term, 2)
    #     else:
    #         obstacles_term += ca.power(conditional_obstacle_term, 2)

    ## adp implementation
    # MIN_D_LONG = 1.0
    # MIN_D_LAT = 1.0
    # obj_length = 1.0
    # obj_width = 1.0
    # ego_length = 1.0
    # ego_width = 1.0
    # # dT = v_ego * tau
    # dLongMin = MIN_D_LONG + 0.5 * obj_length + 0.5 * ego_length # + dT ; positions in geometric center
    # dLatMin = MIN_D_LAT + 0.5 * obj_width + 0.5 * ego_width

    # # TODO: handle more objects
    # dLong = ca.fabs(ocp.model.x[STATE_INDEX_X] - p_obstacles[0])
    # dLat = ca.fabs(ocp.model.x[STATE_INDEX_Y] - p_obstacles[1])

    # aLat = ca.pi / dLatMin # TODO: get dLatMin from param
    # cLat = ca.cos(aLat * dLat) + 1
    # aLong = ca.pi / dLongMin # TODO: get dLongMin from param
    # cLong = ca.cos(aLong * dLong) + 1
    # cObst = cLat * cLong

    # obst_condition = ca.logic_or(dLat > dLatMin, dLong > dLongMin)
    # obstacles_term = ca.if_else(obst_condition, 0, cObst)

    return obstacles_term

def determine_n_discretization_circles(length: ca.MX, width: ca.MX) -> ca.MX:
    # ensure that width > 0.0 for numeric stability
    width = ca.fmax(0.5, width)
    # define aspect_ratio of object
    aspect_ratio = length / width
    # define n_circles based on aspect_ratio
    n_circles = ca.if_else(ca.logic_and(aspect_ratio > 0.0, aspect_ratio <= 2.0), 1,
                 ca.if_else(ca.logic_and(aspect_ratio > 2.0, aspect_ratio <= 4.0), 3,
                 ca.if_else(ca.logic_and(aspect_ratio > 4.0, aspect_ratio <= 6.0), 5,
                 ca.if_else(ca.logic_and(aspect_ratio > 6.0, aspect_ratio <= 8.0), 7,
                 ca.if_else(aspect_ratio > 8.0, 9, 1)))))
    
    return n_circles

def approximate_object_geometry(n_circles: int, x_center: ca.MX, y_center: ca.MX, yaw: ca.MX, length: ca.MX, width: ca.MX):
    # Calculate the radius using symbolic operations
    radius = ca.sqrt(ca.power(length / (2 * n_circles), 2) + ca.power((width / 2.0), 2))

    n_circle_vec = ca.linspace(ca.MX(0), ca.MX(n_circles-1), n_circles)
    circle_centers_x = x_center - (length / 2.0 + (2*n_circle_vec + 1) * length / (2 * n_circles)) * ca.cos(yaw)
    circle_centers_y = y_center - (length / 2.0 + (2*n_circle_vec + 1) * length / (2 * n_circles)) * ca.sin(yaw)
    
    return circle_centers_x, circle_centers_y, radius

def calc_control_cost(ocp: AcadosOcp, config: dict) -> dict:
    j_lon_term = ca.power(ocp.model.u[CONTROL_INDEX_J_LON], 2) / ca.power(config["c_jlon"], 2)
    alpha_term = ca.power(ocp.model.u[CONTROL_INDEX_ALPHA],2 ) / ca.power(config["c_alpha"], 2)

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
    j_lat_term = ca.power(j_lat, 2) / ca.power(config["c_jlat"], 2)

    return j_lat_term
