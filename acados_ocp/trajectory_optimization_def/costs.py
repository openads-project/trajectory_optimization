from acados_template import AcadosOcpCost
from casadi import MX, vertcat, sqrt, if_else, mmin, find, logic_and, logic_not, MX_inf, power, fmax
import numpy as np

def set_costs(ocp, parameter):    

    cost = AcadosOcpCost()

    cost.cost_type = 'NONLINEAR_LS'
    cost.cost_type_e = 'NONLINEAR_LS'

    ref_path = ocp.model.p

    # get nearest path sample
    # Compute distances between each trajectory point and the given point
    squared_diffs = MX(ref_path.rows(), 2)  # Initialize matrix to store squared differences
    squared_diffs[:, 0] = (ref_path[:, 1] - ocp.model.x[0])**2  # (x_i - x)^2 for each i
    squared_diffs[:, 1] = (ref_path[:, 2] - ocp.model.x[1])**2  # (y_i - y)^2 for each i

    # Sum the squared differences and take the square root to get the distance
    distances = sqrt(squared_diffs[:, 0] + squared_diffs[:, 1])
    # Generate vector that stores a 1 at the index of the nearest sample
    min_distance_index_vec = if_else(mmin(distances) == distances[:], 1, 0)
    # Use find to get the nearest sample index
    idx_min = find(min_distance_index_vec)
    # Find nearest adjacent sample
    # Define conditions
    condition_begin = (idx_min == 0)
    condition_end = (idx_min == ref_path.rows()-1)
    condition_intermediate = logic_and(logic_not(condition_begin), logic_not(condition_end))
    # Define expressions for adjacent distances
    dist_1 = if_else(condition_intermediate, distances[idx_min-1], MX_inf(1,1), True)
    dist_2 = if_else(condition_intermediate, distances[idx_min+1], MX_inf(1,1), True)
    condition_dist = (dist_1 < dist_2)
    next_idx_min = if_else(condition_begin, idx_min+1, if_else(condition_end, idx_min-1, if_else(condition_dist, idx_min-1, idx_min+1, True), True), True)
    
    # We now want to compute the shortest distance between the state-point and a line segment idx_min---next_idx_min
    # Extend the segment to a complete line first; determine point with shortest distance to state-point (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line), but formulate as parameter lambda
    # Values [0, 1] for lambda mean the nearest point is on the segment and the computed distance is perpendicular to the line segment
    # Note that lambda must be >=0 due to the way we defined the line segment
    x1 = ref_path[idx_min,1]
    y1 = ref_path[idx_min,2]
    v1 = ref_path[idx_min,3]
    x2 = ref_path[next_idx_min,1]
    y2 = ref_path[next_idx_min,2]
    v2 = ref_path[next_idx_min,3]

    c_square = power(x2-x1,2)+power(y2-y1,2)
    lmd = ((ocp.model.x[0] - x1)*(x2-x1) + (ocp.model.x[1] - y1)*(y2-y1))/ c_square
    x_ref = x1 + lmd * (x2-x1)
    y_ref = y1 + lmd * (y2-y1)
    v_ref = v1 + lmd * (v2-v1)
    dlon = lmd * sqrt(c_square)
    dlat = sqrt(power(ocp.model.x[0]-x_ref,2)+power(ocp.model.x[1]-y_ref,2))

    v_term = (v_ref - ocp.model.x[3]) / fmax(v_ref, 2.78) # (v_ref - v)/max(v_ref, 2.78)
    dlon_term = dlon / 0.2 # 0.2 is a parameter that should be configurable
    dlat_term = dlat / 0.2 # 0.2 is a parameter that should be configurable
    
    ocp.model.cost_y_expr = vertcat(dlat_term, v_term, dlon_term)
    ocp.model.cost_y_expr_0 = vertcat(dlat_term, v_term, dlon_term)
    ocp.model.cost_y_expr_e = vertcat(dlat_term, v_term, dlon_term)

    cost.yref = np.array([0, 0, 0])
    cost.yref_0 = np.array([0, 0, 0])
    cost.yref_e = np.array([0, 0, 0])

    ref_init = np.zeros((parameter['nrefsamples']*4))
    ocp.parameter_values = ref_init

    cost.W = np.diag([1, 1, 0])
    cost.W_0 = np.diag([1, 1, 0])
    cost.W_e = np.diag([1, 1, 0])

    ocp.cost = cost