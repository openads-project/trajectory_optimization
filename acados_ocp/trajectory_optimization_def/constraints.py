from acados_template import AcadosOcpConstraints, AcadosOcp
from casadi import vertcat, tan, atan, cos, sin, fabs, fmax, fmin, sqrt
from utils import stable_tan
from constants import *
import numpy as np

def set_constraints(ocp: AcadosOcp, parameters):

    cons = AcadosOcpConstraints()

    ########## static constraints on state ##########
    # set v_min < v < v_max [m/s]
    # set a_min < a < a_max [m/s^2]
    # set delta_min < delta_f < delta_max [rad]
    # RWS: set delta_min < delta_r < delta_max [rad]

    # constraints on initiall shooting node
    # initial state
    cons.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    if parameters['model_type'] == 'RWS':
        cons.x0 = np.concatenate((cons.x0, [0.0]))

    # constraints on intermediate shooting nodes
    cons.lbx = np.array([parameters['v_min'], -parameters['acceleration_t_max'], -parameters['delta_max']])
    cons.ubx = np.array([parameters['v_max'], parameters['acceleration_t_max'], parameters['delta_max']])
    cons.idxbx = np.array([STATE_INDEX_V_T, STATE_INDEX_A_T, STATE_INDEX_DELTA_F])

    if parameters['model_type'] == 'RWS':
        cons.lbx = np.concatenate((cons.lbx, [-parameters['delta_max']]))
        cons.ubx = np.concatenate((cons.ubx, [parameters['delta_max']]))
        cons.idxbx = np.concatenate((cons.idxbx, [STATE_INDEX_DELTA_R]))

    # constraints on terminal shooting node
    cons.lbx_e = cons.lbx
    cons.ubx_e = cons.ubx
    cons.idxbx_e = cons.idxbx

    ########## static constraints on control ##########
    # set -j_max < j < j_max [m/s^3]
    # set -alpha_max < alpha_f < alpha_max [rad]
    # RWS: set -alpha_max < alpha_r < alpha_max [rad]
    cons.lbu = np.array([-parameters["jerk_max"], -parameters["alpha_max"]])
    cons.ubu = np.array([parameters["jerk_max"], parameters["alpha_max"]])
    cons.idxbu = np.array([CONTROL_INDEX_J_T, CONTROL_INDEX_ALPHA_F])

    if parameters['model_type'] == 'RWS':
        cons.lbu = np.concatenate((cons.lbu, [-parameters['alpha_max']]))
        cons.ubu = np.concatenate((cons.ubu, [parameters['alpha_max']]))
        cons.idxbu = np.concatenate((cons.idxbu, [CONTROL_INDEX_ALPHA_R]))

    ########## nonlinear constraints ##########

    # compute psi_dot:
    # Ackermann: psi_dot = v / l * tan(delta_f)
    # RWS: psi_dot = v * cos(beta) * (tan(delta_f) - tan(delta_r)) / (L_f + L_r)
    #beta = atan((L_r / (L_f + L_r)) * tan(delta_f) + (L_f / (L_f + L_r)) * tan(delta_r))
    psi_dot = None
    if parameters['model_type'] == 'RWS':
        L_f = parameters['distance_cg_front_axle']
        L_r = parameters['distance_cg_rear_axle']
        beta = atan((L_r / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) + (L_f / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_R]))
        psi_dot = ocp.model.x[STATE_INDEX_V_T] * cos(beta) * (stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R])) / (L_f + L_r)
    else:
        l = parameters['wheelbase']
        psi_dot = ocp.model.x[STATE_INDEX_V_T] / l * stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])

    # compute normal acceleration
    a_n = ocp.model.x[STATE_INDEX_V_T] * psi_dot

    # compute absolute acceleration
    a_abs_squared = ocp.model.x[STATE_INDEX_A_T]**2 + (a_n)**2

    # Define nonlinear constraints
    ocp.model.con_h_expr = vertcat(a_abs_squared)
    ocp.model.con_h_expr_e = vertcat(a_abs_squared)

    # Set boundaries for nonlinear constraints
    # a_abs_squared < a_max_squared
    #-psi_dot_max < psi_dot < psi_dot_max
    #-beta_max < beta < beta_max TODO: check if this would improve stability
    a_max_squared = parameters['acceleration_max']**2
    cons.lh = np.array([0.0])
    cons.uh = np.array([a_max_squared])

    if parameters['model_type'] == 'RWS':
        ocp.model.con_h_expr = vertcat(ocp.model.con_h_expr, psi_dot)
        ocp.model.con_h_expr_e = vertcat(ocp.model.con_h_expr_e, psi_dot)
        cons.lh = np.concatenate((cons.lh, [-parameters['psi_dot_max']]))
        cons.uh = np.concatenate((cons.uh, [parameters['psi_dot_max']]))

        # nonlinear constraints for steering mode
        steering_mode_constraints = {
            "in-phase": ocp.model.x[STATE_INDEX_DELTA_F] - ocp.model.x[STATE_INDEX_DELTA_R],
            "anti-phase": ocp.model.x[STATE_INDEX_DELTA_F] + ocp.model.x[STATE_INDEX_DELTA_R]
        }
        if parameters["steering_mode_constraint"] in steering_mode_constraints:
            ocp.model.con_h_expr = vertcat(ocp.model.con_h_expr, steering_mode_constraints[parameters["steering_mode"]])
            ocp.model.con_h_expr_e = vertcat(ocp.model.con_h_expr_e, steering_mode_constraints[parameters["steering_mode"]])
            cons.lh = np.concatenate((cons.lh, [0.0]))
            cons.uh = np.concatenate((cons.uh, [0.0]))
        elif parameters["steering_mode_constraint"] != "none":
            raise ValueError("Invalid steering mode. Choose between 'in-phase', 'anti-phase' or 'none'(default).")

    # also apply same constraints on terminal shooting node
    cons.lh_e = cons.lh
    cons.uh_e = cons.uh

    ########## soft constraints ##########

    if parameters["enable_slack"]:

        # Add slack to nonlinear constraints (idxsh)
        if parameters['model_type'] == 'RWS' and parameters['steering_mode_constraint'] in ['in-phase', 'anti-phase']:
            cons.idxsh = np.array([0, 1, 2])                                    # Index of nonlinear constraints: a_abs_squared, psi_dot and steering_mode_constraint
        elif parameters['model_type'] == 'RWS' and parameters['steering_mode_constraint'] == 'none':
            cons.idxsh = np.array([0, 1])                                       # Index of nonlinear constraints: a_abs_squared and psi_dot
        else:
             cons.idxsh = np.array([0])                                         # Index of nonlinear constraints: a_abs_squared

        # Add slack to state bounds (idxsbx)
        cons.idxsbx = np.array([STATE_INDEX_V_T, STATE_INDEX_A_T])              # Index of state constraints: v_t, a_t
        # In the cost terms, the slack variables are arranged  as follows: idxsbu, idxsbx, idxsg, idxsh
        # Attention: parameters must have the same length as cons.isxsbx + cons.idxsh (depending on the steering mode and model type)
        ocp.cost.Zl = np.diag(parameters["slack_weights"]["quadratic_lower"])   # Quadratic cost on lower bound slack variables
        ocp.cost.Zu = np.diag(parameters["slack_weights"]["quadratic_upper"])   # Quadratic cost on upper bound slack variables
        ocp.cost.zl = np.array(parameters["slack_weights"]["linear_lower"])     # Linear cost on lower bound slack variables
        ocp.cost.zu = np.array(parameters["slack_weights"]["linear_upper"])     # Linear cost on upper bound slack variables

    ocp.constraints = cons
