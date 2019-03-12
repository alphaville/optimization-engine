function run_parametric_controller(build_config)
current_path = pwd();
destination_path = fullfile(build_config.build_path, build_config.build_name);
cd(destination_path);
system('cargo run &');
cd(current_path);