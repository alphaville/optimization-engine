#ifndef ICASADI_HEADER_SENTINEL
#define ICASADI_HEADER_SENTINEL

#include "icasadi_config.h"

extern int CASADI_COST_NAME(
    const double **arg,
    double **casadi_results,
    long long int *iw,
    double *w,
    void *mem);

extern int CASADI_GRAD_NAME(
    const double **arg,
    double **casadi_results,
    long long int *iw,
    double *w,
    void *mem);

extern int CASADI_CONSTRAINTS_AS_PENALTY_NAME(
    const double **arg,
    double **res,
    long long int *iw,
    double *w,
    void *mem);

int icasadi_cost_(
    const double *u,
    const double *casadi_static_params,
    double *cost_value);

int icasadi_grad_(
    const double *u,
    const double *casadi_static_params,
    double *gradient);

int icasadi_constraints_as_penalty_(
    const double *u,
    const double *casadi_static_params,
    double *constraints);

#endif
