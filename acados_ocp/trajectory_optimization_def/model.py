import numpy as np
from acados_template import AcadosModel
from constants import *
from casadi import MX, vertcat, sin, cos, tan

def set_model(ocp, config):

    model = AcadosModel()

    # set model_name
    model.name = config['model_name']

    # set constants
    l = config['wheelbase']

    # set up states
    x      = MX.sym('x')
    y      = MX.sym('y')
    s      = MX.sym('s')
    v      = MX.sym('v')
    a_lon  = MX.sym('a_lon')
    psi    = MX.sym('psi')
    delta  = MX.sym('delta')
    state = vertcat(x, y, s, v, a_lon, psi, delta)

    # set up controls
    j_lon = MX.sym('j_lon')
    alpha = MX.sym('alpha')
    u = vertcat(j_lon, alpha)

    # derivatives
    x_dot = MX.sym('x_dot')
    y_dot = MX.sym('y_dot')
    s_dot = MX.sym('s_dot')
    v_dot = MX.sym('v_dot')
    a_lon_dot = MX.sym('a_lon_dot')
    psi_dot = MX.sym('psi_dot')
    delta_dot = MX.sym('delta_dot')
    state_dot = vertcat(x_dot, y_dot, s_dot, v_dot, a_lon_dot, psi_dot, delta_dot)

    # dynamics
    f_x_dot = state[STATE_INDEX_V] * cos(state[STATE_INDEX_PSI])
    f_y_dot = state[STATE_INDEX_V] * sin(state[STATE_INDEX_PSI])
    f_s_dot = state[STATE_INDEX_V]
    f_v_dot = state[STATE_INDEX_A_LON]
    f_a_lon_dot = u[CONTROL_INDEX_J_LON]
    f_psi_dot = state[STATE_INDEX_V] / l * tan(state[STATE_INDEX_DELTA])
    f_delta_dot = u[CONTROL_INDEX_ALPHA]
    f_expl = vertcat(f_x_dot, f_y_dot, f_s_dot, f_v_dot, f_a_lon_dot, f_psi_dot, f_delta_dot)

    # dynamics: implicit DAE
    model.f_impl_expr = state_dot - f_expl
    model.f_expl_expr = f_expl
    model.x = state
    model.xdot = state_dot
    model.u = u

    # parameters
    p_cost_weights = MX.sym('cost_weights', np.prod(config['p_cost_weights_shape'])) # (nCosts x 1)
    p_ref_path = MX.sym('ref_path', np.prod(config['p_ref_path_shape'])) # (N x (t, x, y, v))
    p_max_vel = MX.sym('max_vel', np.prod(config['p_max_vel_shape'])) # (1 x 1)
    p_s_ref = MX.sym('s_ref', np.prod(config['p_s_ref_shape'])) # (1 x 1)
    p_obstacles = MX.sym('obstacles', np.prod(config['p_obstacles_shape'])) # (nObstacles x (x, y, r))
    params = vertcat(p_cost_weights, p_ref_path, p_max_vel, p_s_ref, p_obstacles)
    model.p = params

    ocp.model = model
