from acados_template import AcadosOcpOptions

def export_opts() -> AcadosOcpOptions:
    
    opts = AcadosOcpOptions()

    opts.tf = 5.0 # prediction horizon s

    return opts