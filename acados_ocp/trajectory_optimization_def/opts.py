from acados_template import AcadosOcpOptions

def set_opts(ocp, config):

    opts = AcadosOcpOptions()

    # set options
    opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM' #'FULL_CONDENSING_QPOASES'
    opts.hessian_approx = "GAUSS_NEWTON"
    opts.integrator_type = "ERK"
    opts.nlp_solver_type = "SQP"
    opts.nlp_solver_max_iter = 100  # default
    opts.nlp_solver_tol_stat = 1e-6 # default
    opts.qp_solver_iter_max = 50    # default
    opts.qp_solver_warm_start = 0   # default
    opts.sim_method_num_stages = 4  # default
    opts.sim_method_num_steps = 1   # default


    # set prediction horizon in s
    opts.tf = config['optimization_horizon']

    ocp.solver_options = opts