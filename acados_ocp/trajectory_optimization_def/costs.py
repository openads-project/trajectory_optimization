from acados_template import AcadosOcpCost
from casadi import vertcat, fmax
import numpy as np

def set_costs(ocp, parameter):    

    cost = AcadosOcpCost()

    cost.cost_type = 'NONLINEAR_LS'
    cost.cost_type_e = 'NONLINEAR_LS'

    v_term = (ocp.model.p[0] - ocp.model.x[3]) / fmax(ocp.model.p[0], 2.78) # (v_ref - v)/max(v_ref, 2.78)

    ocp.parameter_values = np.array([0])

    ocp.model.cost_y_expr = vertcat(ocp.model.x[0], ocp.model.x[1], v_term)
    ocp.model.cost_y_expr_0 = vertcat(ocp.model.x[0], ocp.model.x[1], v_term)
    ocp.model.cost_y_expr_e = vertcat(ocp.model.x[0], ocp.model.x[1], v_term)

    cost.yref = np.array([0, 0, 0])
    cost.yref_0 = np.array([0, 0, 0])
    cost.yref_e = np.array([0, 0, 0])

    cost.W = np.diag([1, 1, 1])
    cost.W_0 = np.diag([1, 1, 1])
    cost.W_e = np.diag([1, 1, 1])

    ocp.cost = cost