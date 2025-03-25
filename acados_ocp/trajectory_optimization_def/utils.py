from casadi import fmax, fmin, tan
import numpy as np

def stable_tan(rad):
    # for numerical stability of the tangent function
    return fmax(-100, fmin(100, tan(rad)))
