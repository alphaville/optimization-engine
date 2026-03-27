#include "icasadi.h"

int icasadi_cost_(
    const double *u,
    const double *casadi_static_params,
    double *cost_value)
{
    const double *casadi_arguments[2];
    casadi_arguments[0] = u;
    casadi_arguments[1] = casadi_static_params;
    return CASADI_COST_NAME(casadi_arguments, &cost_value, 0, 0, 0);
}

int icasadi_grad_(
    const double *u,
    const double *casadi_static_params,
    double *gradient)
{
    const double *casadi_arguments[2];
    casadi_arguments[0] = u;
    casadi_arguments[1] = casadi_static_params;
    return CASADI_GRAD_NAME(casadi_arguments, &gradient, 0, 0, 0);
}

int icasadi_constraints_as_penalty_(
    const double *u,
    const double *casadi_static_params,
    double *constraints)
{
    const double *casadi_arguments[2];
    casadi_arguments[0] = u;
    casadi_arguments[1] = casadi_static_params;
    return CASADI_CONSTRAINTS_AS_PENALTY_NAME(casadi_arguments, &constraints, 0, 0, 0);
}
