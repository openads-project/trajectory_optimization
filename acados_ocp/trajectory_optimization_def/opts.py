from acados_template import AcadosOcpOptions

def set_opts(ocp, config):

    opts = AcadosOcpOptions()

    # set options
    opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    # PARTIAL_CONDENSING_HPIPM      default, works
    # FULL_CONDENSING_QPOASES       diverges
    # FULL_CONDENSING_HPIPM         works, looks very similar to default
    # PARTIAL_CONDENSING_QPDUNES    doesn't work, error message: "qpDUNES interface can not handle ns>0 yet: what about implementing it? :)"
    # PARTIAL_CONDENSING_OSQP       doesn't work, error message: "OSQP interface can not handle ns>0 yet: what about implementing it? :)"
                                        # => ns>0 means, that there are soft constraints. If we remove them, OSQP works, but not faster, QPDUNES diverges.
    # FULL_CONDENSING_DAQP          doesn't work, because some libs cannot be found when building with ACADOS_WITH_DAQP=ON
        
    opts.hessian_approx = "GAUSS_NEWTON"
    opts.integrator_type = "ERK"
    opts.nlp_solver_type = "SQP"
    opts.rti_log_residuals = 1
    opts.nlp_solver_max_iter = 35               # default 100. Bound to ensure real-time capability
    opts.tol = 1e-4                             # default 1e-6
    opts.qp_solver_iter_max = 50                # default 50
    opts.qp_solver_warm_start = 2               # default 0. 1 (warm: Initialize solver primal w/ last it) faster, 2 (hot: also initialize dual) even faster
    opts.qp_tol = 1e-4                          # default None
    opts.sim_method_num_stages = 4              # default 4 -> Use Runge Kutta 4
    opts.sim_method_num_steps = 2               # default 1. Not sure what this does, but 2 seems to make it slightly faster
    opts.globalization = 'FIXED_STEP'           # default. String in ('FIXED_STEP', 'MERIT_BACKTRACKING').
    opts.globalization_use_SOC = 0              # default. 1 could help to solve the problem if 0 fails, but will be slower
    opts.line_search_use_sufficient_descent = 0 # default. 1 could help to solve the problem if 0 fails, but will be slower
    opts.levenberg_marquardt = 0.05             # default. Larger values could help to solve the problem if 0 fails, but will be slower


    # set prediction horizon in s
    opts.tf = config['optimization_horizon']

    ocp.solver_options = opts