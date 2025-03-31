from casadi import fmax, fmin, tan

def stable_tan(rad):
    # for numerical stability of the tangent function
    return fmax(-100, fmin(100, tan(rad)))