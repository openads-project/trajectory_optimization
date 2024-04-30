from acados_template import AcadosOcpOptions

def set_opts(ocp, config):

    opts = AcadosOcpOptions()

    # set options
    opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM' #'FULL_CONDENSING_QPOASES'
    opts.hessian_approx = "GAUSS_NEWTON"
    opts.integrator_type = "ERK"
    opts.nlp_solver_type = "SQP"

    # set prediction horizon in s
    opts.tf = config['optimization_horizon']

    ocp.solver_options = opts