from acados_template import AcadosOcpOptions

def set_opts(ocp, config):

    opts = AcadosOcpOptions()

    # set options
    opts.qp_solver = "PARTIAL_CONDENSING_HPIPM"
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
    opts.timeout_max_time = 100e-3              # default 0 => no timeout [s]
    opts.timeout_heuristic = "MAX_CALL"         # default ZERO. Possible values are MAX_CALL, MAX_OVERALL, LAST, AVERAGE
    opts.nlp_solver_max_iter = 200              # default 100
    opts.nlp_solver_tol_stat = 1e-4
    opts.nlp_solver_tol_eq = 1e-4
    opts.nlp_solver_tol_ineq = 1e-4
    opts.nlp_solver_tol_comp = 1e-4
    opts.tol = 1e-4                             # default 1e-6
    opts.qp_solver_iter_max = 80                # default 50
    opts.qp_solver_warm_start = 2               # default 0. 1 (warm: Initialize solver primal w/ last it) faster, 2 (hot: also initialize dual) even faster
    opts.qp_solver_tol_stat = 1e-4
    opts.qp_solver_tol_eq = 1e-4
    opts.qp_solver_tol_ineq = 1e-4
    opts.qp_solver_tol_comp = 1e-4
    opts.qp_tol = 1e-4                          # default None
    opts.sim_method_num_stages = 4              # default 4 -> Use Runge Kutta 4
    opts.sim_method_num_steps = 2               # default 1. Not sure what this does, but 2 seems to make it slightly faster
    opts.globalization = "MERIT_BACKTRACKING"   # default 'FIXED_STEP'. String in ('FIXED_STEP', 'MERIT_BACKTRACKING'). 'MERIT_BACKTRACKING' more robust than 'FIXED_STEP'
    opts.globalization_use_SOC = 1              # default 0. 1 could help to solve the problem if 0 fails, but will be slower
    opts.line_search_use_sufficient_descent = 1 # default 0. 1 could help to solve the problem if 0 fails, but will be slower
    opts.globalization_line_search_use_sufficient_descent = 1
    opts.levenberg_marquardt = 0.05             # default 0. Larger values improve robustness for nonlinear constraints, but will be slower
    opts.nlp_solver_warm_start_first_qp = True
    opts.nlp_solver_warm_start_first_qp_from_nlp = True

    # set prediction horizon in s
    opts.tf = config['optimization_horizon']

    ocp.solver_options = opts
