/*
 * File: example_optimizer.c
 *
 * Compile with:
 *
 * $ gcc -Wall -std=c99 -pedantic \
    example_optimizer.c -l:lib{{meta.optimizer_name}}.a \
    -L./target/{{build_config.build_mode}} -pthread -lm -ldl \
    -o optimizer
 *
 * OR ... 
 * 
 * $ gcc -Wall -std=c99 -pedantic \
    example_optimizer.c -l{{meta.optimizer_name}} \
    -L./target/{{build_config.build_mode}} -pthread -lm -ldl \
    -o optimizer
 *
 * Or simply do: 
 *  cmake .; make run
 */

#include <stdio.h>
#include "{{meta.optimizer_name}}_bindings.h"

/*
 * Feel free to customize the following code...
 */

int main(void) {
    int i;

    /* parameters             */
    double p[{{meta.optimizer_name|upper}}_NUM_PARAMETERS] = {2.0, 10.0};

    /* initial guess          */
    double u[{{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES] = {0};

    /* initial penalty        */
    double init_penalty = 15.0;

{% if problem.dim_constraints_aug_lagrangian() > 0 %}
    /* initial lagrange mult. */
    double y[{{meta.optimizer_name|upper}}_N1] = {0.0};
{%- endif %}

    /* obtain cache           */
    {{meta.optimizer_name}}Cache *cache = {{meta.optimizer_name}}_new();

    /* solve                  */
    {{meta.optimizer_name}}SolverStatus status = {{meta.optimizer_name}}_solve(cache, u, p, {% if problem.dim_constraints_aug_lagrangian() > 0 %}y{% else %}0{% endif %}, &init_penalty);

    /* print results */
    printf("\n\n-------------------------------------------------\n");
    printf("  Solution\n");
    printf("-------------------------------------------------\n");

    for (i = 0; i < {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES; ++i) {
        printf("u[%d] = %g\n", i, u[i]);
    }

    printf("\n");
    for (i = 0; i < {{meta.optimizer_name|upper}}_N1; ++i) {
        printf("y[%d] = %g\n", i, status.lagrange[i]);
    }

    printf("\n\n-------------------------------------------------\n");
    printf("  Solver Statistics\n");
    printf("-------------------------------------------------\n");
    printf("exit status      : %d\n", status.exit_status);
    printf("iterations       : %lu\n", status.num_inner_iterations);
    printf("outer iterations : %lu\n", status.num_outer_iterations);
    printf("solve time       : %f ms\n", (double)status.solve_time_ns / 1000000.0);
    printf("penalty          : %f\n", status.penalty);
    printf("||Dy||/c         : %f\n", status.delta_y_norm_over_c);
    printf("||F2(u)||        : %f\n", status.f2_norm);
    printf("Cost             : %f\n", status.cost);
    printf("||FRP||          : %f\n\n", status.last_problem_norm_fpr);



    /* free memory */
    {{meta.optimizer_name}}_free(cache);

    return 0;
}

