function open_root = matlab_open_root()

this_file_path = which('matlab_open_root');
matlab_path_str = fileparts(this_file_path);
matlab_path_file = java.io.File(matlab_path_str);
open_root = char(matlab_path_file.getParent());

