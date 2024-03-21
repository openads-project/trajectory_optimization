from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan

def set_model(ocp, parameters) -> AcadosModel:
    
    model = AcadosModel()
    
    # set model_name
    model.name = parameters['model_name']

    # set constants
    l = parameters['wheelbase']

    # set up states
    x      = SX.sym('x')
    y      = SX.sym('y')
    s      = SX.sym('s')
    v      = SX.sym('v')
    a_lon  = SX.sym('a_lon')
    psi    = SX.sym('psi')
    delta  = SX.sym('delta')
    state = vertcat(x, y, s, v, a_lon, psi, delta)

    # set up controls
    j_lon = SX.sym('j_lon')
    alpha = SX.sym('alpha')
    u = vertcat(j_lon, alpha)

    # derivatives
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    s_dot = SX.sym('s_dot')
    v_dot = SX.sym('v_dot')
    a_lon_dot = SX.sym('a_lon_dot')
    psi_dot = SX.sym('psi_dot')
    delta_dot = SX.sym('delta_dot')
    state_dot = vertcat(x_dot, y_dot, s_dot, v_dot, a_lon_dot, psi_dot, delta_dot)

    # dynamics
    f_x_dot = v*cos(psi)
    f_y_dot = v*sin(psi)
    f_s_dot = v
    f_v_dot = a_lon
    f_a_lon_dot = j_lon
    f_psi_dot = v/l*tan(delta)
    f_delta_dot = alpha
    f_expl = vertcat(f_x_dot, f_y_dot, f_s_dot, f_v_dot, f_a_lon_dot, f_psi_dot, f_delta_dot)

    # dynamics: implicit DAE
    model.f_impl_expr = state_dot - f_expl
    model.f_expl_expr = f_expl
    model.x = state
    model.xdot = state_dot
    model.u = u

    ocp.model = model
