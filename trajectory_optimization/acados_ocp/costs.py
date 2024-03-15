from acados_template import AcadosOcpCost

def export_costs() -> AcadosOcpCost:    

    cost = AcadosOcpCost()

    cost.cost_type = 'EXTERNAL'
    cost.cost_type_e = 'EXTERNAL'

    return cost