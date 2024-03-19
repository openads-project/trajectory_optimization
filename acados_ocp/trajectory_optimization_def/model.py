from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_vehicle_model(parameters) -> AcadosModel:
    
    model = AcadosModel()
    
    # set model_name
    model.name = 'trajectory_planning'

    # set constants
    l = 2.711 # wheelbase [m] # To-Do: get from parameters

    # set up states
    x      = SX.sym('x')
    y      = SX.sym('y')
    s      = SX.sym('s')
    v      = SX.sym('v')
    a      = SX.sym('a')
    psi    = SX.sym('psi')
    delta  = SX.sym('delta')
    state = vertcat(x, y, s, v, a, psi, delta)

    # set up controls
    j_lon = SX.sym('j_lon')
    alpha = SX.sym('alpha')
    u = vertcat(j_lon, alpha)

    # derivatives
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    s_dot = SX.sym('s_dot')
    v_dot = SX.sym('v_dot')
    a_dot = SX.sym('a_dot')
    psi_dot = SX.sym('psi_dot')
    delta_dot = SX.sym('delta_dot')
    state_dot = vertcat(x_dot, y_dot, s_dot, v_dot, a_dot, psi_dot, delta_dot)

    # dynamics
    f_x_dot = v*cos(psi)
    f_y_dot = v*sin(psi)
    f_s_dot = v
    f_v_dot = a
    f_a_dot = j_lon
    f_psi_dot = v/l*sin(delta)
    f_delta_dot = alpha
    f_expl = vertcat(f_x_dot, f_y_dot, f_s_dot, f_v_dot, f_a_dot, f_psi_dot, f_delta_dot)

    # dynamics: implicit DAE
    model.f_impl_expr = state_dot - f_expl
    model.f_expl_expr = f_expl
    model.x = state
    model.xdot = state_dot
    model.u = u

    return model
