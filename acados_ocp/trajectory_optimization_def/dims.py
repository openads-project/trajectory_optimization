from acados_template import AcadosOcpDims

def export_dims(model, parameters) -> AcadosOcpDims:    

    dims = AcadosOcpDims()

    dims.N = 50 # prediction horizon (number of intervals) # To-Do get from parameters
    dims.nx = model.x.size()[0] # number of states
    dims.nu = model.u.size()[0] # number of inputs/controls

    return dims