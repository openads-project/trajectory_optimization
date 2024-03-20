from acados_template import AcadosOcpOptions

def set_opts(ocp, parameters) -> AcadosOcpOptions:
    
    opts = AcadosOcpOptions()

    # set options
    opts.qp_solver = 'FULL_CONDENSING_QPOASES'
    opts.nlp_solver_type = "SQP_RTI"
    opts.hessian_approx = "GAUSS_NEWTON"
    opts.integrator_type = "ERK"

    # set prediction horizon in s
    opts.tf = parameters['horizon']

    ocp.solver_options = opts