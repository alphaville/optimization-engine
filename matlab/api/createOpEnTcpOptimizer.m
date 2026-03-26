function client = createOpEnTcpOptimizer(arg1, arg2, varargin)
%CREATEOPENTCPOPTIMIZER creates a MATLAB TCP client for an OpEn optimizer.
%   CLIENT = CREATEOPENTCPOPTIMIZER(PORT) connects to a TCP-enabled
%   generated optimizer on 127.0.0.1:PORT.
%
%   CLIENT = CREATEOPENTCPOPTIMIZER(PORT, IP) connects to the specified
%   IP address and port.
%
%   CLIENT = CREATEOPENTCPOPTIMIZER(IP, PORT) is also accepted.
%
%   CLIENT = CREATEOPENTCPOPTIMIZER(..., Name, Value) forwards all
%   remaining name-value pairs to the OPENTCPOPTIMIZER constructor. See
%   "help OpEnTcpOptimizer" for the supported options and methods.
%
%   This helper keeps the public API lightweight while the implementation
%   lives in the documented OpEnTcpOptimizer class.

    if nargin < 2
        arg2 = [];
    end

    client = OpEnTcpOptimizer(arg1, arg2, varargin{:});
end
