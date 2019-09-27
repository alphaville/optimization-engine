classdef OpEnOptimizer < handle
    %OPENOPTIMIZER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ip;
        port;
        destination_path;
        run_mode;
        udp_connection;
    end
    
    methods
        function o = OpEnOptimizer(ip, port, destination_path, mode)
            o.ip = ip;
            o.port = port;
            if nargin>2, o.destination_path = destination_path; end
            if nargin>3, o.run_mode = mode; end
        end
        
        function run(o)
            % Start the optimizer
            current_path = pwd();
            cd(o.destination_path);
            if strcmp(o.run_mode, 'release')
                system('cargo run --release &');
            else
                system('cargo run &');
            end            
            cd(current_path);
            pause(0.5);
        end
        
        function connect(o)
            o.udp_connection = udp(o.ip, o.port, ...
                'InputBufferSize', 16384, 'OutputBufferSize', 8192);
            fopen(o.udp_connection);
        end
        
        function disconnect(o)
            fclose(o.udp_connection);
        end
        
        function msg = stop(o)
            u = udp(o.ip, o.port);
            fopen(u);
            fwrite(u, 'x');
            X = fread(u, 2048);
            msg = char(X');
            fclose(u);
        end
        
        function out = consume(o, p)
            if length(p) > 1
            p_formatted_str = sprintf('%f, ', p(1:end-1));
            else 
                p_formatted_str = '';
            end
            req_str = sprintf('{"parameter":[%s %f]}', p_formatted_str, p(end));
            fwrite(o.udp_connection, req_str);
            json_response = fread(o.udp_connection, 524288, 'char');
            json_response = char(json_response');
            out = jsondecode(json_response);
        end
    end
    
end

