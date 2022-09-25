import opengen as og
import casadi.casadi as cs
import opengen.functions as fn
from .type_enums import ConstraintMethod


def combine_cartesian_bounds(input_set, bounds_combined, segment_ids, size, horizon):

    offset = segment_ids[-1] if segment_ids else -1

    if isinstance(input_set, list):
        # For list of constraints(dimension: size) of length: horizon
        bounds_combined = bounds_combined + input_set
        segment_ids = segment_ids + [(offset + size * index) for index in range(1, horizon + 1)]
    elif isinstance(input_set, og.constraints.NoConstraints) or input_set.dimension() == size * horizon:
        # For
        #   i. No bounds
        #   ii. Single constraint with dimension: size*horizon
        bounds_combined = bounds_combined + [input_set]
        segment_ids = segment_ids + [offset + size * horizon]
    else:
        # For single constraint with dimension: size
        bounds_combined = bounds_combined + [input_set] * horizon
        segment_ids = segment_ids + [(offset + size * index) for index in range(1, horizon + 1)]
    return bounds_combined, segment_ids


def state_input_constraint(user_ocp, u, x):
    u_set = user_ocp.u_set
    x_set = user_ocp.x_set
    nu = user_ocp.nu
    nx = user_ocp.nx
    N = user_ocp.horizon
    bounds_combined = []
    segment_ids = []

    (bounds_combined, segment_ids) = combine_cartesian_bounds(u_set, bounds_combined, segment_ids, nu, N)
    (bounds_combined, segment_ids) = combine_cartesian_bounds(x_set, bounds_combined, segment_ids, nx, N)

    cartesian_bounds = og.constraints.CartesianProduct(segment_ids, bounds_combined)
    decision_variable = cs.vertcat(u, x)

    return cartesian_bounds, decision_variable


def input_constraint_single_shooting(user_ocp, u):
    u_set = user_ocp.u_set
    nu = user_ocp.nu
    N = user_ocp.horizon
    bounds_combined = []
    segment_ids = []

    (bounds_combined, segment_ids) = combine_cartesian_bounds(u_set, bounds_combined, segment_ids, nu, N)

    cartesian_bounds = og.constraints.CartesianProduct(segment_ids, bounds_combined)

    return cartesian_bounds, u


def set_exclusion_formulation(user_ocp, x):
    exclusion_set_list = user_ocp.exclusion_set
    N = user_ocp.horizon
    nx = user_ocp.nx

    set_exclusion_fn = []

    if exclusion_set_list is None:
        return set_exclusion_fn

    for exclusion_set in exclusion_set_list:
        if not exclusion_set.get_exclusion_mode() is ConstraintMethod.PM:
            raise NotImplementedError("Only Penlaty Method supported for Set Exclusion Constraints")

        set_exclusion_fn = cs.vertcat(set_exclusion_fn, exclusion_set.exclude_set(x, nx, N))

    return set_exclusion_fn


def single_shooting_formulation(user_ocp, p, u, nu, x, nx, horizon):
    cost = 0
    x_t = x[0:nx]
    x_t_buffer = []
    alm_mapping = []
    pm_constraints = []
    bounds_combined = []
    segment_ids = []
    alm_set = None
    x_set = user_ocp.x_set

    for t in range(horizon):
        u_t = u[nu * t:nu * (t + 1)]
        cost += user_ocp.stage_cost_fn(x_t, u_t, p)
        x_t = user_ocp.sys_dyn_fn(x_t, u_t, p)
        for i in range(nx):
            x_t_buffer = cs.vertcat(x_t_buffer, x_t[i])

    cost += user_ocp.terminal_cost_fn(x_t, p)

    (bounds, decision_var) = input_constraint_single_shooting(user_ocp, u)

    if not isinstance(x_set, og.constraints.NoConstraints):
        (bounds_combined, segment_ids) = combine_cartesian_bounds(x_set, bounds_combined, segment_ids, nx, horizon)
        x_cartesian_bounds = og.constraints.CartesianProduct(segment_ids, bounds_combined)
        alm_mapping = cs.vertcat(alm_mapping, x_t_buffer)
        alm_set = x_cartesian_bounds


    set_exclusion_fn = set_exclusion_formulation(user_ocp, x_t_buffer)

    if fn.is_symbolic(set_exclusion_fn):
        pm_constraints = cs.vertcat(pm_constraints, set_exclusion_fn)

    return cost, decision_var, bounds, alm_mapping, alm_set, pm_constraints

def multiple_shooting_formulation(user_ocp, p, u, nu, x, nx, horizon):
    cost = 0
    alm_constraints = []
    pm_constraints = []
    system_dynamics = []
    for t in range(horizon):
        x_t = x[nx * t:nx * (t + 1)]
        x_next = x[nx * (t + 1):nx * (t + 2)]
        u_t = u[nu * t:nu * (t + 1)]

        cost += user_ocp.stage_cost_fn(x_t, u_t, p)

        x_dyn = user_ocp.sys_dyn_fn(x_t, u_t, p)

        for i in range(nx):
            system_dynamics = cs.vertcat(system_dynamics, (x_next[i] - x_dyn[i]))

    x_t = x[nx * horizon:nx * (horizon + 1)]

    cost += user_ocp.terminal_cost_fn(x_t, p)

    (bounds, decision_var) = state_input_constraint(user_ocp, u, x[nx:])

    set_exclusion_fn = set_exclusion_formulation(user_ocp, x[nx:])

    alm_constraints = cs.vertcat(alm_constraints, system_dynamics)
    alm_set = og.constraints.Zero()

    if fn.is_symbolic(set_exclusion_fn):
        pm_constraints = cs.vertcat(pm_constraints, set_exclusion_fn)

    return cost, decision_var, bounds, alm_constraints, alm_set, pm_constraints
