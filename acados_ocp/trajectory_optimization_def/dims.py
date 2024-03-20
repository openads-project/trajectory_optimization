from acados_template import AcadosOcpDims

def set_dims(ocp, parameters) -> AcadosOcpDims:    

    dims = AcadosOcpDims()

    dims.N = parameters['nsteps'] # number of shooting intervals
    dims.nx = ocp.model.x.rows() # number of states
    dims.nu = ocp.model.u.rows() # number of inputs/controls

    ocp.dims = dims