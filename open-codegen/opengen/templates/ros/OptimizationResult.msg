float64[]    solution              # optimizer
uint8        inner_iterations      # number of inner iterations
uint16       outer_iterations      # number of outer iterations
uint8        status                # status code
float64      norm_fpr              # norm of FPR of last inner problem
float64      penalty               # penalty value
float64[]    lagrange_multipliers  # vector of Lagrange multipliers
float64      infeasibility_f1
float64      infeasibility_f2
float64      solve_time_ms

