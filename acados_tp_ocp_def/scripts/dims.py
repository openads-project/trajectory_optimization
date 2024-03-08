from acados_template import AcadosOcpDims

def export_dims() -> AcadosOcpDims:    

    dims = AcadosOcpDims()

    dims.N = 20 # prediction horizon (number of intervals)

    return dims