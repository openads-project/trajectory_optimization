from acados_template import AcadosOcpOptions

def set_opts(ocp, parameters):
    
    opts = AcadosOcpOptions()

    # set options
    opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM' #'FULL_CONDENSING_QPOASES'
    opts.hessian_approx = "GAUSS_NEWTON"
    opts.integrator_type = "IRK"
    opts.nlp_solver_type = "SQP"

    # set prediction horizon in s
    opts.tf = parameters['optimization_horizon']

    ocp.solver_options = opts