from acados_template import AcadosOcpOptions

def export_opts(parameters) -> AcadosOcpOptions:
    
    opts = AcadosOcpOptions()

    # set options
    opts.qp_solver = 'FULL_CONDENSING_QPOASES' # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    opts.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    # opts.print_level = 1
    opts.nlp_solver_type = 'SQP' # SQP_RTI, SQP

    # set prediction horizon in s
    opts.tf = parameters['horizon']

    return opts