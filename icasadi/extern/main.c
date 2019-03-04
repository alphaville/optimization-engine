#include "icasadi.h"
#include <stdio.h>

int main() {
    // long long int n = phi_n_in();
    double u[CASADI_NU] = {1.0, 2.0, 3.0, -5.0, 1.0, 10.0, 14.0, 17.0, 3.0, 5.0};
    double p[CASADI_NP] = {1.0,-1.0};

    double phival = 0;
    double cost_jacobian[CASADI_NU] = {0};

    icasadi_cost_(u, p, &phival);
    icasadi_grad_(u, p, cost_jacobian);
    printf("cost value = %g\n", phival);
    printf("jacobian of cost =\n");
    printf("[\n");
    int i;
    for (i = 0; i < CASADI_NU; ++i){
        printf("  %g\n", cost_jacobian[i]);
    }
    printf("]\n");
    return 0;
}