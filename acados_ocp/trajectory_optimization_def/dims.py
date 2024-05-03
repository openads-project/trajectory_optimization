from acados_template import AcadosOcpDims

def set_dims(ocp, config):

    dims = AcadosOcpDims()

    dims.N = config['n_shots'] # number of shooting intervals
    dims.nx = ocp.model.x.rows() # number of states
    dims.nu = ocp.model.u.rows() # number of inputs/controls
    dims.np = ocp.model.p.rows() # number of model parameters

    ocp.dims = dims