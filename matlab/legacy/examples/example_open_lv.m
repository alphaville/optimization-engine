alpha = 0.6; bet = 0.05; gam = 0.1; 
eta =  0.1; delta = 0.95;  lam = 0.1; 
N = 20; 
q = 1.5; r = 0.1; qN = 2;
xref=0.2; ylim=0.05;

u = casadi.SX.sym('u', N);
z0 = casadi.SX.sym('z0', 2);

x = z0(1); y = z0(2);
cost = 0;
for t = 1:N
    cost = cost + q*(x-xref)^2 + 3e3*max(ylim-y,0) ^2+ r*u(t)^2;    
    x = (alpha*x - bet*x*y)/(1 + gam*x) + u(t);
    y = ( delta*y - eta*x*y)/(1 + lam*y);
end
cost = cost + qN*(x-xref)^2 + 3e3*max(ylim-y,0) ^2;

constraints = OpEnConstraints.make_no_constraints();
builder = OpEnOptimizerBuilder()...
    .with_problem(u, z0, cost, constraints)...
    .with_build_name('lotka_volterra')...
    .with_fpr_tolerance(1e-8);
optimizer = builder.build();


%%
optimizer.run();
optimizer.connect();
z_init = [-1;1];
out = optimizer.consume(z_init);
optimizer.stop();
out
U = out.u;
Z = z_init; x = Z(1); y = Z(2);
for t=1:N
    x = (alpha*x - bet*x*y)/(1 + gam*x) + U(t);
    y = (delta*y - eta*x*y)/(1 + lam*y);
    Z = [Z [x;y]];    
end
figure(2);

subplot(211)
plot(0:N, Z', 'linewidth', 2); ylabel('Population'); grid on;


subplot(212);
plot(0:N-1, out.u, '-k', 'linewidth', 2); ylabel('Actuation');  xlabel('Time');
xlim([0, N]); grid on;