% Minimize ROSENBROCK(u; p)
% subject to: ||u|| <= c

u = casadi.SX.sym('u', 40);        % decision variables
p = casadi.SX.sym('p', 2);         % parameters
phi = rosenbrock(u, p);            % cost function phi(u; p) (Rosenbrock)
constraints = OpEnConstraints.make_ball_at_origin(5.0);
opEnBuilder = OpEnOptimizerBuilder().with_problem(u, p, phi, constraints);
opEnOptimizer = opEnBuilder.build();

%% Consume
opEnOptimizer.run();
opEnOptimizer.connect();
out = opEnOptimizer.consume([1,2]);
opEnOptimizer.stop();


disp(out);