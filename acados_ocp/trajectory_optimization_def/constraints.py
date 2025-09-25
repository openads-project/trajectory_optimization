from acados_template import AcadosOcpConstraints, AcadosOcp
import casadi as ca
from utils import stable_tan, determine_spacially_matched_ref_path_point, approximate_ego_geometry
from constants import *
import numpy as np

def set_constraints(ocp: AcadosOcp, config):

    cons = AcadosOcpConstraints()

    ########## static constraints on state ##########
    # set v_min < v < v_max [m/s]
    # set a_min < a < a_max [m/s^2]
    # set delta_min < delta_f < delta_max [rad]
    # RWS: set delta_min < delta_r < delta_max [rad]

    # constraints on initiall shooting node
    # initial state
    cons.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    if config['model_type'] == 'RWS':
        cons.x0 = np.concatenate((cons.x0, [0.0]))

    # constraints on intermediate shooting nodes
    cons.lbx = np.array([config['v_min'], -config['acceleration_t_max'], -config['delta_max']])
    cons.ubx = np.array([config['v_max'], config['acceleration_t_max'], config['delta_max']])
    cons.idxbx = np.array([STATE_INDEX_V_T, STATE_INDEX_A_T, STATE_INDEX_DELTA_F])

    if config['model_type'] == 'RWS':
        cons.lbx = np.concatenate((cons.lbx, [-config['delta_max']]))
        cons.ubx = np.concatenate((cons.ubx, [config['delta_max']]))
        cons.idxbx = np.concatenate((cons.idxbx, [STATE_INDEX_DELTA_R]))

    # constraints on terminal shooting node
    cons.lbx_e = cons.lbx
    cons.ubx_e = cons.ubx
    cons.idxbx_e = cons.idxbx


    ########## static constraints on control ##########
    # set -j_max < j < j_max [m/s^3]
    # set -alpha_max < alpha_f < alpha_max [rad]
    # RWS: set -alpha_max < alpha_r < alpha_max [rad]
    cons.lbu = np.array([-config["jerk_max"], -config["alpha_max"]])
    cons.ubu = np.array([config["jerk_max"], config["alpha_max"]])
    cons.idxbu = np.array([CONTROL_INDEX_J_T, CONTROL_INDEX_ALPHA_F])

    if config['model_type'] == 'RWS':
        cons.lbu = np.concatenate((cons.lbu, [-config['alpha_max']]))
        cons.ubu = np.concatenate((cons.ubu, [config['alpha_max']]))
        cons.idxbu = np.concatenate((cons.idxbu, [CONTROL_INDEX_ALPHA_R]))


    ########## nonlinear constraints ##########
    ocp.model.con_h_expr = ca.vertcat()  # initialize empty expression for nonlinear constraints
    cons.lh = np.array([])  # initialize empty lower bounds for nonlinear constraints
    cons.uh = np.array([])  # initialize empty upper bounds for nonlinear constraints

    ### route boundaries ###

    # get ref path with boundaries from global parameters
    idx_global_params = 0
    p_cost_weights = ocp.model.p_global[idx_global_params:(idx_global_params := idx_global_params + np.prod(config["p_cost_weights_shape"]))]
    p_cost_params = ocp.model.p_global[idx_global_params:(idx_global_params := idx_global_params + np.prod(config["p_cost_params_shape"]))]
    p_ref_path = ocp.model.p_global[idx_global_params:(idx_global_params := idx_global_params + np.prod(config["p_ref_path_shape"]))]
    assert idx_global_params == np.prod(config["p_cost_weights_shape"]) + np.prod(config["p_cost_params_shape"]) + np.prod(config["p_ref_path_shape"])
    d_min_boundary_lat = p_cost_params[3]

    # approximate the ego-vehicle with circles
    ego_approximation = approximate_ego_geometry(ocp, config)

    # calc normal vector from interpolated reference point to the current position
    ref_inter = determine_spacially_matched_ref_path_point(config, p_ref_path, ocp.model.x[STATE_INDEX_X], ocp.model.x[STATE_INDEX_Y])
    normal_vec = ca.vertcat(-ca.sin(ref_inter["psi"]), ca.cos(ref_inter["psi"]))

    # calc signed lateral offset from interpolated reference path to current position
    vec_x = ocp.model.x[STATE_INDEX_X] - ref_inter["x"]
    vec_y = ocp.model.x[STATE_INDEX_Y] - ref_inter["y"]
    ref_diff = ca.vertcat(vec_x, vec_y)
    d_normal = ca.dot(ref_diff, normal_vec)

    # calc offset to boundaries for each ego circle
    MAX_OFFSET_BOUNDARY = 1e6  # large value to "disable" boundary constraint to one side (could not use inf, because of numerical issues)
    for i in range(config["n_ego_circles"]):

        # circle offset from current position
        circle_offset = ca.vertcat(ego_approximation["x_offset"][i], ego_approximation["y_offset"][i])
        
        # distance to reference from each ego circle center
        d_circle_center_ref_path = d_normal + ca.dot(circle_offset, normal_vec)

        # constraints for left and right road boundary
        # left boundary: countouring_error - d_left_boundary + (r + d_min_boundary_lat) < 0
        # right boundary: countouring_error + d_right_boundary - (r + d_min_boundary_lat) > 0
        # assumption: d_left_boundary + d_right_boundary >= 2 * (r + d_min_boundary_lat)
        assumption_check = (ref_inter["d_left_boundary"] + ref_inter["d_right_boundary"] < 2 * (ego_approximation["radius"] + d_min_boundary_lat))
        boundary_constraint_left = ca.if_else(assumption_check, -MAX_OFFSET_BOUNDARY-1, d_circle_center_ref_path - ref_inter["d_left_boundary"] + ego_approximation["radius"] + d_min_boundary_lat)
        boundary_constraint_right = ca.if_else(assumption_check, MAX_OFFSET_BOUNDARY+1, d_circle_center_ref_path + ref_inter["d_right_boundary"] - ego_approximation["radius"] - d_min_boundary_lat)

        ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, boundary_constraint_left, boundary_constraint_right)
        cons.lh = np.concatenate((cons.lh, [-MAX_OFFSET_BOUNDARY, 0.0]))  # large negagtive number to "disable" left boundary constraint for right offset
        cons.uh = np.concatenate((cons.uh, [0.0, MAX_OFFSET_BOUNDARY]))   # large positive number to "disable" right boundary constraint for left offset

    ### a_abs_squared, psi_dot, steering_mode_constraint ###
    # Ackermann: psi_dot = v / l * tan(delta_f)
    # RWS: psi_dot = v * cos(beta) * (tan(delta_f) - tan(delta_r)) / (L_f + L_r)
    # beta = atan((L_r / (L_f + L_r)) * tan(delta_f) + (L_f / (L_f + L_r)) * tan(delta_r))
    psi_dot = None
    if config['model_type'] == 'RWS':
        L_f = config['distance_cg_front_axle']
        L_r = config['distance_cg_rear_axle']
        beta = ca.atan((L_r / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) + (L_f / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_R]))
        psi_dot = ocp.model.x[STATE_INDEX_V_T] * ca.cos(beta) * (stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R])) / (L_f + L_r)
    else:
        l = config['wheelbase']
        psi_dot = ocp.model.x[STATE_INDEX_V_T] / l * stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])

    # compute normal acceleration
    a_n = ocp.model.x[STATE_INDEX_V_T] * psi_dot

    # Set boundaries for nonlinear constraints
    # a_abs_squared < a_max_squared
    #-psi_dot_max < psi_dot < psi_dot_max
    #-beta_max < beta < beta_max TODO: check if this would improve stability

    # compute absolute acceleration
    a_abs_squared = ocp.model.x[STATE_INDEX_A_T]**2 + (a_n)**2
    ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, a_abs_squared)
    a_max_squared = config['acceleration_max']**2
    cons.lh = np.concatenate((cons.lh, [0.0]))
    cons.uh = np.concatenate((cons.uh, [a_max_squared]))

    if config['model_type'] == 'RWS':
        ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, psi_dot)
        cons.lh = np.concatenate((cons.lh, [-config['psi_dot_max']]))
        cons.uh = np.concatenate((cons.uh, [config['psi_dot_max']]))

        # nonlinear constraints for steering mode
        steering_mode_constraints = {
            "in-phase": ocp.model.x[STATE_INDEX_DELTA_F] - ocp.model.x[STATE_INDEX_DELTA_R],
            "anti-phase": ocp.model.x[STATE_INDEX_DELTA_F] + ocp.model.x[STATE_INDEX_DELTA_R]
        }
        if config["steering_mode_constraint"] in steering_mode_constraints:
            ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, steering_mode_constraints[config["steering_mode_constraint"]])
            cons.lh = np.concatenate((cons.lh, [0.0]))
            cons.uh = np.concatenate((cons.uh, [0.0]))
        elif config["steering_mode_constraint"] != "none":
            raise ValueError("Invalid steering mode. Choose between 'in-phase', 'anti-phase' or 'none'(default).")

    # also apply same constraints on terminal shooting node
    ocp.model.con_h_expr_e = ocp.model.con_h_expr
    cons.lh_e = cons.lh
    cons.uh_e = cons.uh

    ########## soft constraints ##########

    if config["enable_slack"]:

        # Add slack only to route bounds (nonlinear constraints (idxsh))
        cons.idxsh = np.arange(0, config["n_ego_circles"] * 2)  # Index of nonlinear constraints with slack: left and right boundary for each ego circle

        # Add slack to state bounds (idxsbx)
        # cons.idxsbx = np.array([STATE_INDEX_V_T, STATE_INDEX_A_T])              # Index of state constraints: v_t, a_t -> disabled

        # In the cost terms, the slack variables are arranged  as follows: idxsbu, idxsbx, idxsg, idxsh
        # Attention: parameters must have the same length as cons.isxsbx + cons.idxsh
        ocp.cost.Zl = np.diag(config["slack_weights"]["quadratic_lower"])   # Quadratic cost on lower bound slack variables
        ocp.cost.Zu = np.diag(config["slack_weights"]["quadratic_upper"])   # Quadratic cost on upper bound slack variables
        ocp.cost.zl = np.array(config["slack_weights"]["linear_lower"])     # Linear cost on lower bound slack variables
        ocp.cost.zu = np.array(config["slack_weights"]["linear_upper"])     # Linear cost on upper bound slack variables

    ocp.constraints = cons
