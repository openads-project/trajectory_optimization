from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_vehicle_model() -> AcadosModel:
    
    model = AcadosModel()
    
    # set model_name
    model.name = 'kinematic_single_track_model'

    # set constants
    l = 2.711 # wheelbase [m]

    # set up states
    x1      = SX.sym('x1')
    y1      = SX.sym('y1')
    s1      = SX.sym('s1')
    v1      = SX.sym('v1')
    a1      = SX.sym('a1')
    psi1    = SX.sym('psi1')
    delta1  = SX.sym('delta1')
    x = vertcat(x1, y1, s1, v1, a1, psi1, delta1)

    # set up controls
    j_lon = SX.sym('j_lon')
    alpha = SX.sym('alpha')
    u = vertcat(j_lon, alpha)

    # derivatives
    x1_dot = SX.sym('x1_dot')
    y1_dot = SX.sym('y1_dot')
    s1_dot = SX.sym('s1_dot')
    v1_dot = SX.sym('v1_dot')
    a1_dot = SX.sym('a1_dot')
    psi1_dot = SX.sym('psi1_dot')
    delta1_dot = SX.sym('delta1_dot')
    xdot = vertcat(x1_dot, y1_dot, s1_dot, v1_dot, a1_dot, psi1_dot, delta1_dot)

    # dynamics
    f_x1_dot = v1*cos(psi1)
    f_y1_dot = v1*sin(psi1)
    f_s1_dot = v1
    f_v1_dot = a1
    f_a1_dot = j_lon
    f_psi1_dot = v1/l*sin(delta1)
    f_delta1_dot = alpha
    f_expl = vertcat(f_x1_dot, f_y1_dot, f_s1_dot, f_v1_dot, f_a1_dot, f_psi1_dot, f_delta1_dot)

    # dynamics: implicit DAE
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u

    return model