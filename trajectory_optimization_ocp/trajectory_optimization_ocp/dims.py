# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

from acados_template import AcadosOcpDims


def set_dims(ocp, config):
    """Set dimensions for the OCP problem based on configuration.

    Args:
        ocp: The ACADOS OCP object to configure.
        config: Configuration dictionary containing optimization parameters.
    """
    dims = AcadosOcpDims()

    dims.N = config["n_shots"]  # number of shooting intervals
    dims.nx = ocp.model.x.rows()  # number of states
    dims.nu = ocp.model.u.rows()  # number of inputs/controls
    dims.np = ocp.model.p.rows()  # number of model parameters
    dims.np_global = ocp.model.p_global.rows()  # number of model global parameters

    ocp.dims = dims
