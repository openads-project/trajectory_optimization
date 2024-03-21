from acados_template import AcadosOcpOptions

def set_opts(ocp, parameters):
    
    opts = AcadosOcpOptions()

    # set options
    opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM' #'FULL_CONDENSING_QPOASES'
    opts.nlp_solver_type = "SQP_RTI"
    opts.hessian_approx = "GAUSS_NEWTON"
    opts.integrator_type = "ERK"

    # set prediction horizon in s
    opts.tf = parameters['horizon']

    ocp.solver_options = opts