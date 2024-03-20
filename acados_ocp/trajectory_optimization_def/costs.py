from acados_template import AcadosOcpCost

def set_costs(ocp, parameter) -> AcadosOcpCost:    

    cost = AcadosOcpCost()

    cost.cost_type = 'EXTERNAL'
    cost.cost_type_e = 'EXTERNAL'

    # cost.cost_type = 'NONLINEAR_LS'
    # cost.cost_type_e = 'LINEAR_LS'

    ocp.cost = cost