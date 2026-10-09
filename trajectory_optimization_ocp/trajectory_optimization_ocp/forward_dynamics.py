# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import casadi as ca
import numpy as np
from constants import CONTROL_INDEX_J_T, STATE_INDEX_A_T, STATE_INDEX_DELTA_F, STATE_INDEX_V_T


def forward_acceleration(velocity, acceleration):
    """Apply a longitudinal acceleration demand without driving backwards at standstill."""
    return ca.if_else(velocity > 0.0, acceleration, ca.fmax(acceleration, 0.0))


def _integrate_motion(dynamics, state, control, duration):
    """Integrate a moving segment with exact longitudinal states and two RK4 steps."""
    state = ca.MX(state)
    step = duration / 2.0
    jerk = control[CONTROL_INDEX_J_T]
    for _ in range(2):
        velocity = state[STATE_INDEX_V_T]
        acceleration = state[STATE_INDEX_A_T]
        midpoint_velocity = velocity + acceleration * step / 2.0 + jerk * step**2 / 8.0
        end_velocity = velocity + acceleration * step + jerk * step**2 / 2.0

        k1 = dynamics(state, control)
        midpoint = state + step / 2.0 * k1
        midpoint[STATE_INDEX_V_T] = midpoint_velocity
        k2 = dynamics(midpoint, control)
        midpoint = state + step / 2.0 * k2
        midpoint[STATE_INDEX_V_T] = midpoint_velocity
        k3 = dynamics(midpoint, control)
        endpoint = state + step * k3
        endpoint[STATE_INDEX_V_T] = end_velocity
        k4 = dynamics(endpoint, control)
        state += step / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        state[STATE_INDEX_V_T] = end_velocity
    return state


def set_forward_discrete_dynamics(ocp, config):
    """Discretize forward motion, splitting intervals at stopping and brake release.

    The acceleration state remains the longitudinal acceleration/braking demand,
    with its rate limited by the jerk control. Negative demand holds a stopped
    vehicle; positive jerk can release the brake and restart within the interval.
    Append the interval duration to stage parameters to retain runtime horizons.

    Args:
        ocp: Configured OCP with continuous bicycle dynamics and parameter defaults.
        config: Model configuration containing the default shooting interval.
    """
    model = ocp.model
    duration = ca.MX.sym("time_step")
    model.p = ca.vertcat(model.p, duration)
    ocp.parameter_values = np.append(ocp.parameter_values, config["optimization_horizon"] / config["n_shots"])
    dynamics = ca.Function("bicycle_dynamics", [model.x, model.u], [model.f_expl_expr])

    initial = ca.MX(model.x)
    initial[STATE_INDEX_V_T] = ca.fmax(initial[STATE_INDEX_V_T], 0.0)
    velocity = initial[STATE_INDEX_V_T]
    acceleration = initial[STATE_INDEX_A_T]
    jerk = model.u[CONTROL_INDEX_J_T]

    # Check the minimum, also detecting stop/restart with a positive endpoint.
    nonzero_jerk = ca.if_else(jerk != 0.0, jerk, 1.0)
    minimum_time = ca.if_else(jerk > 0.0, ca.fmin(ca.fmax(-acceleration / nonzero_jerk, 0.0), duration), duration)
    minimum_velocity = ca.fmin(velocity, velocity + acceleration * minimum_time + jerk * minimum_time**2 / 2.0)
    stops = minimum_velocity < 0.0

    # Stable descending root of v + a*t + j*t^2/2 = 0. Keep inactive branches
    # finite as CasADi also differentiates them, including at zero velocity/jerk.
    discriminant = acceleration**2 - 2.0 * jerk * velocity
    root = ca.sqrt(ca.if_else(discriminant > 0.0, discriminant, 1.0))
    denominator = -acceleration + root
    stop_time = ca.if_else(
        acceleration > 0.0,
        (-acceleration - root) / nonzero_jerk,
        2.0 * velocity / ca.if_else(denominator != 0.0, denominator, 1.0),
    )
    stop_time = ca.if_else(ca.logic_and(velocity == 0.0, acceleration <= 0.0), 0.0, stop_time)
    stop_time = ca.fmin(ca.fmax(stop_time, 0.0), duration)
    release_time = ca.if_else(jerk > 0.0, ca.fmin(ca.fmax(-acceleration / nonzero_jerk, stop_time), duration), duration)

    stopped = _integrate_motion(dynamics, initial, model.u, stop_time)
    stopped[STATE_INDEX_V_T] = 0.0
    hold_time = release_time - stop_time
    stopped[STATE_INDEX_A_T] += hold_time * jerk
    # Steering can change while holding the brake, without moving the vehicle.
    for index in range(STATE_INDEX_DELTA_F, int(model.x.numel())):
        stopped[index] += hold_time * model.u[index - STATE_INDEX_DELTA_F + 1]
    restarted = _integrate_motion(dynamics, stopped, model.u, duration - release_time)
    moving = _integrate_motion(dynamics, initial, model.u, duration)
    next_state = ca.if_else(stops, restarted, moving, True)
    next_state[STATE_INDEX_V_T] = ca.fmax(next_state[STATE_INDEX_V_T], 0.0)
    model.disc_dyn_expr = next_state
