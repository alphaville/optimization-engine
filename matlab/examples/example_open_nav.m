% parameters
L = 0.5; ts = 0.1;

% Prediction horizon
N = 50; 

% target point and bearing

% weights
q = 10; qtheta = 2; r = 20;
qN = 100*q; qthetaN = 50*qtheta;

nu = 2; nx = 3;
u = casadi.SX.sym('u', nu*N); 
p = casadi.SX.sym('p', 2*nx+1);
x = p(1); y = p(2); theta = p(3);
xref = p(4); yref = p(5); thetaref=p(6);
h_penalty = p(end);
cost = 0;
zref = [xref; yref];

% Obstacle (disc centered at `zobs` with radius `c`)
c = 0.4; zobs = [0.7; 0.5];

for t = 1:nu:nu*N    
    z = [x; y];
    cost = cost + q*norm(z-zref) + qtheta*(theta-thetaref)^2 ;         
    cost = cost + h_penalty * max(c^2 - norm(z-zobs)^2, 0)^2;
    u_t = u(t:t+1);
    theta_dot = (1/L)*(u_t(2)*cos(theta) - u_t(1)*sin(theta));
    cost = cost + r*(u_t'*u_t);
    x = x + ts * (u_t(1) + L * sin(theta) * theta_dot);
    y = y + ts * (u_t(2) - L * cos(theta) * theta_dot);
    theta = theta + ts * theta_dot;
end
cost = cost + qN*((x-xref)^2 + (y-yref)^2) + qthetaN*(theta-thetaref)^2;
cost = cost + h_penalty * max(c^2 - norm(z-zobs)^2, 0)^2;

constraints = OpEnConstraints.make_no_constraints();
builder = OpEnOptimizerBuilder()...
    .with_problem(u, p, cost, constraints)...
    .with_build_name('navigation')...
    .with_build_mode('release')...
    .with_fpr_tolerance(1e-4)...
    .with_lbfgs_memory(25)...
    .with_max_iterations(1000);
optimizer = builder.build();

%%
optimizer.run();
optimizer.connect();

%% Draw circle
circ = [];
for theta = 0:0.01:2*pi
    circ = [circ zobs + c*[sin(theta); cos(theta)]];
end
plot(circ(1,:), circ(2,:), 'k', 'linewidth', 2)
hold on; axis equal;
%%
z_init = [0.7; -0.3; deg2rad(50)];
z_ref  = [0.7;  1.1; deg2rad(120)];
weight_obstacle = 8000;
out = optimizer.consume([z_init; z_ref; weight_obstacle]);
out


% Simulate
Z = z_init(1:3); x = Z(1); y = Z(2); theta = Z(3);
U = out.u;
for t=1:2:2*N    
    u_t = U(t:t+1);
    theta_dot = (1/L)*(u_t(2)*cos(theta) - u_t(1)*sin(theta));
    x = x + ts * (u_t(1) + L * sin(theta) * theta_dot);
    y = y + ts * (u_t(2) - L * cos(theta) * theta_dot);
    theta = theta + ts * theta_dot;
    Z = [Z [x;y;theta]];    
end

%figure;
% subplot(311); plot(reshape(out.u,  2, N)'); grid on;
if out.n < 1000
plot(Z(1,:), Z(2,:), '-', 'linewidth', 2); grid on; hold on;
end
% subplot(313); plot(Z(3,:)); grid on;


flushoutput( optimizer.udp_connection);
flushinput( optimizer.udp_connection)
%%
optimizer.disconnect();
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
plot(z_ref(1), z_ref(2), 'r+', 'linewidth', 3)
grid on;
xlabel('x');
ylabel('y');
legend('trajectory','target')

%%
plot(0:N, rad2deg(Z(3,:)), 'linewidth', 2);
grid on;
xlabel('Time instant');
ylabel('theta (deg)');