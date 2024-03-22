from acados_template import AcadosOcpDims

def set_dims(ocp, parameters):    

    dims = AcadosOcpDims()

    dims.N = parameters['nsteps'] # number of shooting intervals
    dims.nx = ocp.model.x.rows() # number of states
    dims.nu = ocp.model.u.rows() # number of inputs/controls
    dims.np = ocp.model.p.rows() # number of model parameters
    dims.ny = ocp.model.cost_y_expr.rows() # number of reference inputs
    dims.ny_0 = ocp.model.cost_y_expr.rows() # number of reference inputs
    dims.ny_e = ocp.model.cost_y_expr_e.rows() # number of reference inputs at terminal state

    ocp.dims = dims