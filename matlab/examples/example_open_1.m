% Minimize ROSENBROCK(u; p)
% subject to: ||u|| <= c
clc;
nu = 5; np = 3;
u = casadi.SX.sym('u', nu);             % decision variables
p = casadi.SX.sym('p', np);             % parameters
phi = rosenbrock(u, p(1:2));            % cost function phi(u; p) (Rosenbrock)
constraints_type_p = [2*norm(u) - p(3); % c(u, p)
                      5*u - 3;
                      3 - 5*u];
nc = length(constraints_type_p);

mu = casadi.SX.sym('mu', nc);
q = [p; mu];

F = phi + mu'*constraints_type_p;
F_fun = casadi.Function('F', {u, q}, {F});


F_fun(ones(nu,1), ones(np+nc,1))

Fgrad = gradient(F, u);
Fgrad_fun = casadi.Function('Fgrad', {u, q}, {Fgrad});

Fgrad_fun(ones(nu,1), ones(np+nc,1))
%%



constraints = OpEnConstraints.make_ball_at_origin(5.0);

opEnBuilder = OpEnOptimizerBuilder()...
    .with_problem(u, p, phi, constraints)...
    .with_build_name('penalty_new')...
    .with_fpr_tolerance(1e-3)...
    .with_constraints_as_penalties(constraints_type_p);

opEnOptimizer = opEnBuilder.build();

%% Consume
clc;
opEnOptimizer.run();
opEnOptimizer.connect();
out = opEnOptimizer.consume([0.5,2,1])
out = opEnOptimizer.consume([0.5,2,1])
opEnOptimizer.stop();

