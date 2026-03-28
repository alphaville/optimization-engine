%SETUP_OPEN should be executed once to set up the Optimization Engine
%toolbox.
%

open_root_path = matlab_open_root();

open_matlab_path = fullfile(open_root_path, 'matlab');
addpath(genpath(open_matlab_path));

savepath;

fprintf('CONGRATULATIONS! You have successfully set up Optimization Engine\n');