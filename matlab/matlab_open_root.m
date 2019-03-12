function rd = matlab_open_root()

rd = which('matlab_open_root');
tokens = strsplit(rd, 'matlab/matlab_open_root.m');
rd = tokens{1};