/*
 * Auto-generated header file
 * This file is part of OptimizationEngine
 * (see https://alphaville.github.io/optimization-engine/)
 *
 * Generated at: 2019-05-30 21:26:40.351228
 */
#ifndef ICASADI_CONFIG_HEADER_SENTINEL
#define ICASADI_CONFIG_HEADER_SENTINEL

/* Global constant definitions */

/* Number of decision variables */
#define CASADI_NU 5

/* Number of parameters */
#define CASADI_NP 2

/* Number of constraints to be treated using the penalty method */
#define CASADI_NUM_CONSTAINTS_TYPE_PENALTY 3

/* Name of cost function */
#define CASADI_COST_NAME phi

/* Name of the gradient of the cost function */
#define CASADI_GRAD_NAME grad_phi

/* Name of c(x) -- constraints as penalties */
#define CASADI_CONSTRAINTS_AS_PENALTY_NAME constraints_penalty

/* Name of c'(x) -- constraints for AL */
#define CASADI_CONSTRAINTS_AL_NAME constraints_aug_lagrangian

#endif