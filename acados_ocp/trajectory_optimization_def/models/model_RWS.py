import numpy as np
from acados_template import AcadosModel
from constants import *
from casadi import MX, vertcat, sin, cos, atan
from utils import stable_tan

def set_model(ocp, config):
    """
    Set up bicycle model with front and rear wheel steering
    referenced at the center of gravtiy
    """

    model = AcadosModel()

    # set model_name
    model.name = config['model_name']

    # set constants
    L_f = config['distance_cg_front_axle']
    L_r = config['distance_cg_rear_axle']

    # set up states -> x = [x, y, s, v_t, a_t, psi, delta_f, delta_r,]
    x = MX.sym('x')
    y = MX.sym('y')
    s = MX.sym('s')
    v_t = MX.sym('v_t')
    a_t = MX.sym('a_t')
    psi = MX.sym('psi')
    delta_f = MX.sym('delta_f')
    delta_r = MX.sym('delta_r')

    state = vertcat(x, y, s, v_t, a_t, psi, delta_f, delta_r)

    # set up controls -> u = [j_t, delta_f_dot, delta_r_dot]
    j_t = MX.sym('j_t')
    alpha_f = MX.sym('alpha_f')
    alpha_r = MX.sym('alpha_r')

    u = vertcat(j_t, alpha_f, alpha_r)

    # derivates -> x_dot = f(x, u) = [x_dot, y_dot, s_dot, v_t_dot, a_t_dot, psi_dot, alpha_f, alpha_r]
    x_dot = MX.sym('x_dot')
    y_dot = MX.sym('y_dot')
    s_dot = MX.sym('s_dot')
    v_t_dot = MX.sym('v_t_dot')
    a_t_dot = MX.sym('a_t_dot')
    psi_dot = MX.sym('psi_dot')
    alpha_f = MX.sym('alpha_f')
    alpha_r = MX.sym('alpha_r')

    state_dot = vertcat(x_dot, y_dot, s_dot, v_t_dot, a_t_dot, psi_dot, alpha_f, alpha_r)

    # intermediate variable: vehicle side slip angle (depends on system states delta_f, delta_r)
    beta = atan((L_r / (L_f + L_r)) * stable_tan(state[STATE_INDEX_DELTA_F]) + (L_f / (L_f + L_r)) * stable_tan(state[STATE_INDEX_DELTA_R]))

    # dynamics : explicit DAE
    f_x_dot = state[STATE_INDEX_V_T] * cos(state[STATE_INDEX_PSI] + beta)
    f_y_dot = state[STATE_INDEX_V_T] * sin(state[STATE_INDEX_PSI] + beta)
    f_s_dot = state[STATE_INDEX_V_T]
    f_v_t_dot = state[STATE_INDEX_A_T]
    f_a_t_dot = u[CONTROL_INDEX_J_T]
    f_psi_dot = state[STATE_INDEX_V_T] * cos(beta) * (stable_tan(state[STATE_INDEX_DELTA_F]) - stable_tan(state[STATE_INDEX_DELTA_R])) / (L_f + L_r)
    f_alpha_f = u[CONTROL_INDEX_ALPHA_F]
    f_alpha_r = u[CONTROL_INDEX_ALPHA_R]

    f_expl = vertcat(f_x_dot, f_y_dot, f_s_dot, f_v_t_dot, f_a_t_dot, f_psi_dot, f_alpha_f, f_alpha_r)
    model.f_expl_expr = f_expl

    # dynamics: implicit DAE
    model.f_impl_expr = state_dot - f_expl

    # set model
    model.x = state
    model.xdot = state_dot
    model.u = u

    # parameters
    p_dynamic_weight = MX.sym('dynamic_weight', np.prod(config['p_dynamic_weight_shape'])) # 1
    p_ref_point = MX.sym('ref_point', np.prod(config['p_ref_point_shape'])) # (x, y, v) -> 3
    p_obstacles = MX.sym('obstacles', np.prod(config['p_obstacle_circles_shape'])) # (nObstacleCircles x (x, y, radius))
    params = vertcat(p_dynamic_weight, p_ref_point, p_obstacles)
    model.p = params

    # global parameters
    p_cost_weights = MX.sym('cost_weights', np.prod(config['p_cost_weights_shape'])) # (nCosts x 1)
    p_cost_params = MX.sym('cost_params', np.prod(config['p_cost_params_shape'])) # (thw, d_min_obstacle_long, d_min_obstacle_lat, d_min_boundary_lat) -> 4
    p_ref_path = MX.sym('ref_path', np.prod(config['p_ref_path_shape'])) # (N x (psi, x, y, v, d_bound_left, d_bound_right)) -> (N x 6)
    global_params = vertcat(p_cost_weights, p_cost_params, p_ref_path)
    model.p_global = global_params

    ocp.model = model