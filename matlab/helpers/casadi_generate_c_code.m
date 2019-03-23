function [phi_fun, jphi_fun] = casadi_generate_c_code(u, p, phi, build_directory, function_name)
%CASADI_GENERATE_C_CODE generates code for a given function and its
%Jacobian
%   
%Syntax:
% [phi_fun, jphi_fun] = casadi_generate_c_code(u, p, phi, fun_name)
%
%Input arguments:
% u           casadi symbol for u
% p           casadi symbol for p
% phi         casadi function phi (cost function)
% fun_name    desired function name (default: 'phi')

if nargin < 5, function_name = 'phi'; end
build_icasadi_directory  = fullfile(build_directory, 'icasadi');

nu = length(u); np = length(p);

cost_fname = function_name;
grad_fname = strcat('grad_', cost_fname);
c_extension = '.c';
cost_c_file = strcat('auto_casadi_cost', c_extension);
grad_c_file = strcat('auto_casadi_grad', c_extension);

cost_c_path = fullfile(build_icasadi_directory, 'extern', cost_c_file);
grad_c_path = fullfile(build_icasadi_directory, 'extern', grad_c_file);

jphi = jacobian(phi, u);
phi_fun = casadi.Function(cost_fname, {u, p}, {phi});
jphi_fun = casadi.Function(grad_fname, {u, p}, {jphi});

phi_fun.generate(cost_c_file);
movefile(cost_c_file, cost_c_path)

jphi_fun.generate(grad_c_file);
movefile(grad_c_file, grad_c_path)

% -------------------------------------------------------------------------
% Generate Header File 
% -------------------------------------------------------------------------
casadi_interface_destination = fullfile(build_icasadi_directory, 'extern', 'icasadi_config.h');
fid = fopen(casadi_interface_destination, 'w');

fprintf(fid, '/* Auto-generated header file */\n');
fprintf(fid, '#ifndef ICASADI_CONFIG_HEADER_SENTINEL\n');
fprintf(fid, '#define ICASADI_CONFIG_HEADER_SENTINEL\n\n');

fprintf(fid, '#define CASADI_NU %d\n', nu);
fprintf(fid, '#define CASADI_NP %d\n', np);
fprintf(fid, '#define CASADI_COST_NAME %s\n', cost_fname);
fprintf(fid, '#define CASADI_GRAD_NAME %s\n\n', grad_fname);

fprintf(fid, '#endif\n');

fclose(fid);

