% Minimize f(u; p)
% subject to: u in U

nu = 40;                           % number of decision variables
np = 2;                            % number of parameters 
u = casadi.SX.sym('u', nu);        % decision variables
p = casadi.SX.sym('p', np);        % parameters
phi = rosenbrock(u, p);            % cost function phi(u; p) (Rosenbrock)

constraints = OpEnConstraints.make_ball_at_origin(30.0);

build_config = open_build_config();
%build_config.target = 'rpi';       % build for Raspberry Pi
build_config.solver.tolerance = 1e-5;
build_config.build_mode = 'release';
build_config.udp_interface.bind_address = '0.0.0.0';
build_config.udp_interface.port = 3498;

open_generate_code(build_config, constraints, u, p, phi);

%% Start the Module
run_parametric_optimizer(build_config);


%% Open connection
%ip_address = '192.168.2.2';
%TODO: the input/output buffer sizes are not design parameters! They should
%be computed as functions of `nu` and `np`; the same holds for the
%communication buffer in Rust. Alternatively, keep small buffers and
%iterate till the end of the message ;)
ip_address = '127.0.0.1';
udp_connection = udp(ip_address, build_config.udp_interface.port, ...
    'InputBufferSize', 16384, ...
    'OutputBufferSize', 4096);
fopen(udp_connection);

%% Use the module
param = randn(np,1);
x = []; dt = [];
N = 40;
tic;
for i=1:N    
    param(1) = param(1) - 10;  
    param(2) = param(2) + 5;
    out = consume_parametric_optimizer(udp_connection, param);
    flushinput(udp_connection);
    flushoutput(udp_connection);
    %out
end
fprintf('Mean time = %.2f\n', toc*1000/N)
%% Stop the connection
fclose(udp_connection);
delete(udp_connection);
clear udp_connection

%% Kill the module
kill_parametric_optimizer(ip_address, build_config.udp_interface.port);