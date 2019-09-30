/*
 * Deprecated - to be removed in the future
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "the_optimizer1_bindings.h"
#include "the_optimizer2_bindings.h"

#define TEST_EPS (1e-16)

int main() {
    double p1[THE_OPTIMIZER1_NUM_PARAMETERS] = {1.0, 10.0};
    double u1[THE_OPTIMIZER1_NUM_DECISION_VARIABLES] = {0};

    the_optimizer1Cache *cache1 = the_optimizer1_new();
    the_optimizer1SolverStatus status1 = the_optimizer1_solve(cache1, u1, p1);
    the_optimizer1_free(cache1);

    printf("\n--- Solver 1 running ---\n\n");
    for (int i = 0; i < THE_OPTIMIZER1_NUM_DECISION_VARIABLES; ++i) {
        printf("u[%d] = %g\n", i, u1[i]);
    }

    printf("exit status = %d\n", status1.exit_status);
    printf("iterations = %lu\n", status1.num_inner_iterations);
    printf("outer iterations = %lu\n", status1.num_outer_iterations);

    double p2[THE_OPTIMIZER2_NUM_PARAMETERS] = {1.0, 10.0};
    double u2[THE_OPTIMIZER2_NUM_DECISION_VARIABLES] = {0};

    the_optimizer2Cache *cache2 = the_optimizer2_new();
    the_optimizer2SolverStatus status2 = the_optimizer2_solve(cache2, u2, p2);
    the_optimizer2_free(cache2);

    printf("\n--- Solver 2 running ---\n\n");
    for (int i = 0; i < THE_OPTIMIZER2_NUM_DECISION_VARIABLES; ++i) {
        printf("u[%d] = %g\n", i, u2[i]);
    }

    printf("exit status = %d\n", status2.exit_status);
    printf("iterations = %lu\n", status2.num_inner_iterations);
    printf("outer iterations = %lu\n", status2.num_outer_iterations);

    /* TESTS */
    printf("\n--- Testing results: Solver 1 == Solver 2 ---\n");

    assert(THE_OPTIMIZER1_NUM_DECISION_VARIABLES == THE_OPTIMIZER2_NUM_DECISION_VARIABLES);
    assert(THE_OPTIMIZER1_NUM_PARAMETERS == THE_OPTIMIZER2_NUM_PARAMETERS);
    for (int i = 0; i < THE_OPTIMIZER2_NUM_DECISION_VARIABLES; ++i) {
        assert(fabs(u1[i] - u2[i]) < TEST_EPS);
    }
    assert(THE_OPTIMIZER1_NUM_DECISION_VARIABLES == THE_OPTIMIZER2_NUM_DECISION_VARIABLES);
    assert(THE_OPTIMIZER1_NUM_DECISION_VARIABLES == THE_OPTIMIZER2_NUM_DECISION_VARIABLES);
    assert(THE_OPTIMIZER1_NUM_DECISION_VARIABLES == THE_OPTIMIZER2_NUM_DECISION_VARIABLES);
    assert(status1.num_inner_iterations == status2.num_inner_iterations);
    assert(status1.num_outer_iterations == status2.num_outer_iterations);

    printf("\nTest Ok!\n\n");

    return 0;
}

