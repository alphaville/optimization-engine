classdef OpEnOptimizer
    %OPENOPTIMIZER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ip;
        port;
        destination_path;
        udp_connection;
    end
    
    methods
        function o = OpEnOptimizer(ip, port, destination_path)
            o.ip = ip;
            o.port = port;
            if nargin>2, o.destination_path = destination_path; end
        end
        
        function run(o)
        end
        
        function stop(o)
        end
        
        function consume(o, p)
        end
    end
    
end

