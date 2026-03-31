function client = createOpEnTcpOptimizer(varargin)
%CREATEOPENTCPOPTIMIZER Create a MATLAB TCP client for an OpEn optimizer.
%   CLIENT = CREATEOPENTCPOPTIMIZER(PORT) connects to a TCP-enabled
%   generated optimizer on 127.0.0.1:PORT.
%
%   CLIENT = CREATEOPENTCPOPTIMIZER(PORT, IP) connects to the specified
%   IP address and port.
%
%   CLIENT = CREATEOPENTCPOPTIMIZER(IP, PORT) is also accepted.
%
%   CLIENT = CREATEOPENTCPOPTIMIZER('ManifestPath', MANIFESTPATH) creates
%   a manifest-aware OCP TCP client and tries to read the endpoint from the
%   sibling ``optimizer.yml`` file.
%
%   CLIENT = CREATEOPENTCPOPTIMIZER(..., Name, Value) forwards all
%   remaining name-value pairs to the OPENTCPOPTIMIZER constructor. See
%   "help OpEnTcpOptimizer" for the supported options and methods.

    client = OpEnTcpOptimizer(varargin{:});
end
