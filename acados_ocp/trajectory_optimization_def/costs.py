import numpy as np
from acados_template import AcadosOcpCost, AcadosOcp
from constants import *
import casadi as ca
from utils import stable_tan, determine_spacially_matched_ref_path_point, approximate_ego_geometry, wrap_angle

def set_costs(ocp: AcadosOcp, config):

    # set up as external cost function
    cost = AcadosOcpCost()
    cost.cost_type = "EXTERNAL"
    cost.cost_type_0 = "EXTERNAL"
    cost.cost_type_e = "EXTERNAL"
    ocp.cost = cost

    # initialize parameters
    n_params_dynamic_weight = np.prod(config["p_dynamic_weight_shape"])
    n_params_obstacles = np.prod(config["p_obstacle_circles_shape"])
    # total number of parameters
    n_params =  n_params_dynamic_weight + n_params_obstacles
    #  set initial parameter values
    ocp.parameter_values = np.zeros(n_params)
    # get parameters
    idx_params = 0
    p_dynamic_weight = ocp.model.p[idx_params:(idx_params := idx_params + n_params_dynamic_weight)]
    p_obstacles = ocp.model.p[idx_params:(idx_params := idx_params + n_params_obstacles)]
    assert idx_params == n_params

    # initialize global parameters
    n_params_cost_weights = np.prod(config["p_cost_weights_shape"])
    n_params_cost_params = np.prod(config["p_cost_params_shape"])
    n_params_ref_path = np.prod(config["p_ref_path_shape"])
    # total number of global parameters
    n_global_params = n_params_cost_weights + n_params_cost_params + n_params_ref_path
    #  set initial global parameter values
    ocp.p_global_values = np.zeros(n_global_params)
    # get global parameters
    idx_global_params = 0
    p_cost_weights = ocp.model.p_global[idx_global_params:(idx_global_params := idx_global_params + n_params_cost_weights)]
    p_cost_params = ocp.model.p_global[idx_global_params:(idx_global_params := idx_global_params + n_params_cost_params)]
    p_ref_path = ocp.model.p_global[idx_global_params:(idx_global_params := idx_global_params + n_params_ref_path)]
    assert idx_global_params == n_global_params

    # cost term weights
    w_lat = p_cost_weights[0]
    w_psi = p_cost_weights[1]
    w_v_t_inter = p_cost_weights[2]
    w_obstacles = p_cost_weights[3]
    w_a_n = p_cost_weights[4]
    w_a_t_pos = p_cost_weights[5]
    w_a_t_neg = p_cost_weights[6]
    w_j_n = p_cost_weights[7]
    w_j_t_pos = p_cost_weights[8]
    w_j_t_neg = p_cost_weights[9]
    w_alpha = p_cost_weights[10]
    w_end_yaw = p_cost_weights[11]

    # other cost params
    p_thw = p_cost_params[0]
    d_min_obstacle_long = p_cost_params[1]
    d_min_obstacle_lat = p_cost_params[2]
    d_min_boundary_lat = p_cost_params[3]

    ########## external cost function ##########

    # calculate quantities needed for cost terms
    interpolated_state_ref = determine_spacially_matched_ref_path_point(config, p_ref_path, ocp.model.x[STATE_INDEX_X], ocp.model.x[STATE_INDEX_Y])
    ego_approximation = approximate_ego_geometry(ocp, config)

    # calculate cost terms
    ref_path_costs = calc_ref_path_cost(ocp, config, interpolated_state_ref)
    obstacles_costs = calc_obstacles_cost(ocp, config, p_obstacles, ego_approximation, p_thw, d_min_obstacle_long, d_min_obstacle_lat)
    a_costs = calc_acceleration_cost(ocp, config)
    control_costs = calc_control_cost(ocp, config)

    #### define costs at intermediate nodes
    # reference path costs
    ocp.model.cost_expr_ext_cost = p_dynamic_weight * w_lat * ref_path_costs["dlat"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_v_t_inter * ref_path_costs["v_t_inter"]
    # obstacle costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_obstacles * obstacles_costs
    # acceleration costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_a_t_pos * a_costs["a_t_pos"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_a_t_neg * a_costs["a_t_neg"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_a_n * a_costs["a_n"]
    # control variable costs
    ocp.model.cost_expr_ext_cost += w_j_t_pos * control_costs["j_t_pos"]
    ocp.model.cost_expr_ext_cost += w_j_t_neg * control_costs["j_t_neg"]
    ocp.model.cost_expr_ext_cost += w_j_n * control_costs["j_n"]
    ocp.model.cost_expr_ext_cost += w_alpha * control_costs["alpha_f"]
    # cost exclusivly for RWS
    if config["model_type"] == "RWS":
        ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_psi * ref_path_costs["psi"]
        ocp.model.cost_expr_ext_cost += w_alpha * control_costs["alpha_r"]

    #### define costs at initial shooting node
    ocp.model.cost_expr_ext_cost_0 = ocp.model.cost_expr_ext_cost

    #### define costs at terminal shooting node
    ocp.model.cost_expr_ext_cost_e = w_end_yaw * ref_path_costs["psi"]


def calc_ref_path_cost(ocp: AcadosOcp, config: dict, ref_inter: dict) -> dict:

    # lateral deviation term
    dlat = ca.sqrt(ca.power(ocp.model.x[STATE_INDEX_X] - ref_inter["x"], 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - ref_inter["y"], 2))
    dlat_term = ca.power(dlat, 2) / ca.power(config["c_lat"], 2)

    # v deviation term
    # first ensure that the reference velocity is > 0
    v_ref = ca.fmax(ref_inter["v"], 0.0)
    # we define the scaling value of v to v_ref: a velocity deviation of v_ref leads to a cost of 1
    # for numeric stability (low reference speeds) we ensure that v_scale > V_SCALE_MIN > 0
    v_scale = ca.fmax(v_ref, V_SCALE_MIN)
    v_term = ca.power((v_ref - ocp.model.x[STATE_INDEX_V_T]), 2) / ca.power(v_scale, 2)

    # psi deviation term
    psi_term = ca.power(wrap_angle(ocp.model.x[STATE_INDEX_PSI] - ref_inter["psi"]), 2) / ca.power(config["c_psi"], 2)

    cost_terms = {"dlat": dlat_term, "psi": psi_term, "v_t_inter": v_term}
    return cost_terms

def calc_obstacles_cost(ocp: AcadosOcp, config: dict, p_obstacles: ca.MX, ego_circles: dict, p_thw: ca.MX, d_min_obstacle_long: ca.MX, d_min_obstacle_lat: ca.MX) -> ca.MX:

    obstacles_term = ca.MX(0.0)
    obstacle_state_dim = config["p_obstacle_circles_shape"][1]
    n_obstacle_circles = config["p_obstacle_circles_shape"][0]
    for i in range(n_obstacle_circles):
        x_center = p_obstacles[i * obstacle_state_dim + P_OBSTACLES_INDEX_X]
        y_center = p_obstacles[i * obstacle_state_dim + P_OBSTACLES_INDEX_Y]
        r_circle = p_obstacles[i * obstacle_state_dim + P_OBSTACLES_INDEX_RADIUS]

        # initialize closest distances to object with a large value
        closest_distance = ca.inf
        dLong = ca.inf
        dLat = ca.inf

        # find the ego-circle that gives the closest distance to the object-circle and store dLat and dLon
        for j in range(config["n_ego_circles"]):
            dx = x_center - ego_circles["x"][j]
            dy = y_center - ego_circles["y"][j]
            # determine dLong and dLat wrt. idx_min
            gamma = ca.atan2(dy, dx)
            alpha = wrap_angle(gamma - ocp.model.x[STATE_INDEX_PSI])
            c = ca.sqrt(ca.power(dx, 2) + ca.power(dy, 2))
            # update the minimum dLong and dLat value if c < closest_distance
            dLong = ca.if_else(c < closest_distance, c * ca.cos(alpha), dLong)
            dLat = ca.if_else(c < closest_distance, c * ca.sin(alpha), dLat)
            closest_distance = ca.if_else(c < closest_distance, c, closest_distance)

        # define minimum lateral and longitudinal distance to object circles
        beta = compute_side_slip_angle(ocp, config)

        x = ca.fabs(p_thw * ocp.model.x[STATE_INDEX_V_T] * ca.cos(beta)) # TODO: verify if this is correct
        y = ca.fabs(p_thw * ocp.model.x[STATE_INDEX_V_T] * ca.sin(beta)) # TODO: verify if this is correct
        dLatMin = ca.fmax(d_min_obstacle_lat, y) + ego_circles["radius"] + r_circle
        dLongMin = ca.fmax(d_min_obstacle_long, x) + ego_circles["radius"] + r_circle

        # calculate cost for object-circle that shows the minimum distance to the ego-vehicle-circle
        cLong = ca.cos((ca.fabs(dLong) / dLongMin) * ca.pi) + 1
        cLat = ca.cos((ca.fabs(dLat) / dLatMin) * ca.pi) + 1
        cObst = cLat * cLong
        no_obst_condition = ca.logic_or(ca.fabs(dLat) > dLatMin, ca.fabs(dLong) > dLongMin)
        obstacles_term += ca.if_else(no_obst_condition, 0, cObst)

    return obstacles_term

def calc_control_cost(ocp: AcadosOcp, config: dict) -> dict:
    j_t_pos = ca.fmax(0, ocp.model.u[CONTROL_INDEX_J_T])
    j_t_pos_term = ca.power(j_t_pos, 2) / ca.power(config["c_j_t"], 2)
    j_t_neg = ca.fmin(0, ocp.model.u[CONTROL_INDEX_J_T])
    j_t_neg_term = ca.power(j_t_neg, 2) / ca.power(config["c_j_t"], 2)

    # derive nominal jerk j_n = d(a_n)/dt =d(v_t * psi_dot)/dt = a_t * psi_dot + v_t * psi_ddot (second derivative)
    psi_dot = (compute_psi_dot_RWS(ocp, config) if config["model_type"] == "RWS" else compute_psi_dot_Ack(ocp, config))
    psi_ddot = (compute_psi_ddot_RWS(ocp, config) if config["model_type"] == "RWS" else compute_psi_ddot_Ack(ocp, config))

    j_n = ocp.model.x[STATE_INDEX_A_T] * psi_dot + ocp.model.x[STATE_INDEX_V_T] * psi_ddot
    j_n_term = ca.power(j_n, 2) / ca.power(config["c_j_n"], 2)

    alpha_f_term = ca.power(ocp.model.u[CONTROL_INDEX_ALPHA_F], 2) / ca.power(config["c_alpha"], 2)
    alpha_r_term = (compute_alpha_r_cost_RWS(ocp, config) if config["model_type"] == "RWS" else 0.0)

    cost_terms = {"j_t_pos": j_t_pos_term, "j_t_neg": j_t_neg_term, "j_n": j_n_term,"alpha_f": alpha_f_term, "alpha_r": alpha_r_term}
    return cost_terms

def calc_acceleration_cost(ocp: AcadosOcp, config: dict) -> dict:

    # tangential acceleration is given as state variable
    a_t_pos = ca.fmax(0, ocp.model.x[STATE_INDEX_A_T])
    a_t_pos_term = ca.power(a_t_pos, 2) / ca.power(config["c_a_t"], 2)
    a_t_neg = ca.fmin(0, ocp.model.x[STATE_INDEX_A_T])
    a_t_neg_term = ca.power(a_t_neg, 2) / ca.power(config["c_a_t"], 2)

    # derive nominal acceleration a_n = psi_dot * v
    psi_dot = (compute_psi_dot_RWS(ocp, config) if config["model_type"] == "RWS" else compute_psi_dot_Ack(ocp, config))
    a_n = psi_dot * ocp.model.x[STATE_INDEX_V_T]
    a_n_term = ca.power(a_n, 2) / ca.power(config["c_a_n"], 2)

    cost_terms = {"a_n": a_n_term, "a_t_pos": a_t_pos_term, "a_t_neg": a_t_neg_term}
    return cost_terms


########## helper functions ##########

def compute_side_slip_angle(ocp: AcadosOcp, config: dict) -> ca.MX:
    """
    Computes the vehicle side slip (angle between vehicle frame and absolute velocity vector)
    Ackermann:
    beta = 0.0
    RWS:
    beta = atan( L_r / (L_f + L_r) * delta_f +  L_f / (L_f + L_r) * delta_r)
    """
    if config["model_type"] == "Ackermann":
        return 0.0
    # RWS
    L_f = config["distance_cg_front_axle"]
    L_r = config["distance_cg_rear_axle"]
    beta = ca.atan((L_r / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) + (L_f / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_R]))
    return beta

def compute_alpha_r_cost_RWS(ocp: AcadosOcp, config: dict) -> ca.MX:
    alpha_r_term = ca.power(ocp.model.u[CONTROL_INDEX_ALPHA_R], 2) / ca.power(config["c_alpha"], 2)
    return alpha_r_term

def compute_psi_dot_RWS(ocp: AcadosOcp, config: dict) -> ca.MX:
    """
    Computes the yaw rate for for a vehicle with RWS:
    psi_dot = v_t * cos(beta) * (tan(delta_f) - tan(delta_r)) / (L_f + L_r)
    """
    L_f = config["distance_cg_front_axle"]
    L_r = config["distance_cg_rear_axle"]
    beta = compute_side_slip_angle(ocp, config)
    psi_dot = ocp.model.x[STATE_INDEX_V_T] * ca.cos(beta) * (stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R])) / (L_f + L_r)
    return psi_dot

def compute_psi_dot_Ack(ocp: AcadosOcp, config: dict) -> ca.MX:
    """
    Computes the yaw rate for a vehicle with Ackermann steering
    psi_dot = v_t / L * tan(delta)
    """
    L = config['wheelbase']
    tan_delta = stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])
    psi_dot = ocp.model.x[STATE_INDEX_V_T] / L * tan_delta
    return psi_dot

def compute_psi_ddot_RWS(ocp: AcadosOcp, config: dict) -> ca.MX:
    """
    Computes the second derivative of yaw angle (yaw acceleration) for RWS:
    ψ̈ = (∂ψ̇/∂v) * ∂v/∂t + (∂ψ̇/∂β) * (∂β/∂t) + (∂ψ̇/∂δ_f) * (∂δ_f/∂t) + (∂ψ̇/∂δ_r) * (∂δ_r/∂t)
    """
    L_f = config["distance_cg_front_axle"]
    L_r = config["distance_cg_rear_axle"]
    beta = compute_side_slip_angle(ocp, config)
    # Compute beta_dot (rate of change of beta which is a function of delta_f and delta_r)
    beta_dot = (1 / (1 + ((L_r * ca.tan(ocp.model.x[STATE_INDEX_DELTA_F]) +
                        L_f * ca.tan(ocp.model.x[STATE_INDEX_DELTA_R])) / (L_f + L_r))**2) *
                ((L_r * ocp.model.u[CONTROL_INDEX_ALPHA_F] / ca.cos(ocp.model.x[STATE_INDEX_DELTA_F])**2 +
                L_f * ocp.model.u[CONTROL_INDEX_ALPHA_R] / ca.cos(ocp.model.x[STATE_INDEX_DELTA_R])**2) / (L_f + L_r)))

    # Compute partial derivatives
    d_psi_dot_d_v = ocp.model.x[STATE_INDEX_A_T] * ca.cos(beta) * (
                        stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R])) / (L_f + L_r)

    d_psi_dot_d_beta = (-ocp.model.x[STATE_INDEX_V_T] * ca.sin(beta) *
                        (stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R])) / (L_f + L_r))

    d_psi_dot_d_delta_f = (ocp.model.x[STATE_INDEX_V_T] * ca.cos(beta) *
                            ocp.model.u[CONTROL_INDEX_ALPHA_F] / (ca.cos(ocp.model.x[STATE_INDEX_DELTA_F])**2) / (L_f + L_r))

    d_psi_dot_d_delta_r = (-ocp.model.x[STATE_INDEX_V_T] * ca.cos(beta) *
                            ocp.model.u[CONTROL_INDEX_ALPHA_R] / (ca.cos(ocp.model.x[STATE_INDEX_DELTA_R])**2) / (L_f + L_r))

    # Compute final yaw acceleration
    yaw_ddot = d_psi_dot_d_v + d_psi_dot_d_beta * beta_dot + d_psi_dot_d_delta_f + d_psi_dot_d_delta_r
    return yaw_ddot

def compute_psi_ddot_Ack(ocp: AcadosOcp, config: dict) -> ca.MX:
    """
    Computes the second derivative of yaw angle (yaw acceleration) for Ackermann steering:
    ψ̈ = (∂ψ̇/∂v) * ∂v/∂t + (∂ψ̇/∂δ_f) * (∂δ_f/∂t)
    """
    L = config['wheelbase']
    tan_delta = stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])
    psi_ddot = (ocp.model.x[STATE_INDEX_A_T] / L * tan_delta +
                ocp.model.x[STATE_INDEX_V_T] / L * ocp.model.u[CONTROL_INDEX_ALPHA_F] * (1 + tan_delta**2))
    return psi_ddot