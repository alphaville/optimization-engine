# Constants match the enumeration of status codes
uint8 STATUS_CONVERGED=0
uint8 STATUS_NOT_CONVERGED_ITERATIONS=1
uint8 STATUS_NOT_CONVERGED_OUT_OF_TIME=2
uint8 STATUS_NOT_CONVERGED_COST=3
uint8 STATUS_NOT_CONVERGED_FINITE_COMPUTATION=4

float64[]    solution              # optimizer (solution)
uint8        inner_iterations      # number of inner iterations
uint16       outer_iterations      # number of outer iterations
uint8        status                # status code
float64      cost                  # cost at solution
float64      norm_fpr              # norm of FPR of last inner problem
float64      penalty               # penalty value
float64[]    lagrange_multipliers  # vector of Lagrange multipliers
float64      infeasibility_f1      # infeasibility wrt F1
float64      infeasibility_f2      # infeasibility wrt F2
float64      solve_time_ms         # solution time in ms

