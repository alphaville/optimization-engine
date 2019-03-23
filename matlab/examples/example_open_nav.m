% parameters
L = 0.5; ts = 0.1;

% Prediction horizon
N = 50; 

% target point and bearing
xref=1; yref=1; thetaref = 0;

% weights
q = 10; qtheta = .1; r = 1;
qN = 10*q; qthetaN = 10*qtheta;

nu = 2; nx = 3;
u = casadi.SX.sym('u', nu*N); 
z0 = casadi.SX.sym('z0', nx);

x = z0(1); y = z0(2); theta = z0(3);
cost = 0;
for t = 1:nu:nu*N
    cost = cost + q*((x-xref)^2 + (y-yref)^2) + qtheta*(theta-thetaref)^2 ;  
    u_t = u(t:t+1);
    theta_dot = (1/L)*(u_t(2)*cos(theta) - u_t(1)*sin(theta));
    cost = cost + r*(u_t'*u_t);
    x = x + ts * (u_t(1) + L * sin(theta) * theta_dot);
    y = y + ts * (u_t(2) - L * cos(theta) * theta_dot);
    theta = theta + ts * theta_dot;
end
cost = cost + qN*((x-xref)^2 + (y-yref)^2) + qthetaN*(theta-thetaref)^2;

constraints = OpEnConstraints.make_no_constraints();
builder = OpEnOptimizerBuilder()...
    .with_problem(u, z0, cost, constraints)...
    .with_build_name('navigation')...
    .with_build_mode('release')...
    .with_fpr_tolerance(1e-4)...
    .with_max_iterations(1000);
optimizer = builder.build();

%%
optimizer.run();
optimizer.connect();
%%
z_init = [0.55;0.25;deg2rad(80)];
out = optimizer.consume(z_init);
out


% Simulate
Z = z_init; x = Z(1); y = Z(2); theta = Z(3);
U = out.u;
for t=1:2:2*N    
    u_t = U(t:t+1);
    theta_dot = (1/L)*(u_t(2)*cos(theta) - u_t(1)*sin(theta));
    x = x + ts * (u_t(1) + L * sin(theta) * theta_dot);
    y = y + ts * (u_t(2) - L * cos(theta) * theta_dot);
    theta = theta + ts * theta_dot;
    Z = [Z [x;y;theta]];    
end

figure;
subplot(311); plot(reshape(out.u,  2, N)'); grid on;
subplot(312); plot(Z(1,:), Z(2,:)); grid on;
subplot(313); plot(Z(3,:)); grid on;

%%
optimizer.stop();

%%
plot(reshape(out.u,  2, N)', 'linewidth', 2);
grid on;
xlabel('Time instant');
ylabel('Position (x,y)');
legend('x-position', 'y-position')

%%
plot(Z(1,:), Z(2,:), 'linewidth', 2);
hold on;
plot(xref, yref, 'r+', 'linewidth', 3)
grid on;
xlabel('x');
ylabel('y');
legend('trajectory','target')

%%
plot(0:N, rad2deg(Z(3,:)), 'linewidth', 2);
grid on;
xlabel('Time instant');
ylabel('theta (deg)');