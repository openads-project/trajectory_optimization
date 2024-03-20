from acados_template import AcadosOcpDims

def export_dims(model, parameters) -> AcadosOcpDims:    

    dims = AcadosOcpDims()

    dims.N = parameters['nsteps'] # number of shooting intervals
    dims.nx = model.x.rows() # number of states
    dims.nu = model.u.rows() # number of inputs/controls

    return dims