classdef OpEnTcpOptimizer < handle
    %OPENTCPOPTIMIZER TCP client for Python-generated OpEn optimizers.
    %   CLIENT = OPENTCPOPTIMIZER(PORT) creates a client that connects to a
    %   TCP-enabled optimizer running on 127.0.0.1:PORT.
    %
    %   CLIENT = OPENTCPOPTIMIZER(PORT, IP) connects to the optimizer at
    %   the specified IP address and TCP port.
    %
    %   CLIENT = OPENTCPOPTIMIZER(IP, PORT) is also accepted for callers
    %   who prefer to provide the endpoint in IP/port order.
    %
    %   CLIENT = OPENTCPOPTIMIZER(..., 'Timeout', T) sets the socket
    %   connect/read timeout in seconds.
    %
    %   CLIENT = OPENTCPOPTIMIZER(..., 'MaxResponseBytes', N) limits the
    %   maximum number of bytes accepted from the optimizer.
    %
    %   This interface is intended for optimizers generated in Python with
    %   the TCP interface enabled. The optimizer server must already be
    %   running; this class only communicates with it.
    %
    %   Example:
    %       client = OpEnTcpOptimizer(3301);
    %       response = client.solve([2.0, 10.0]);
    %       if response.ok
    %           disp(response.solution);
    %       else
    %           error('OpEn:RemoteError', '%s', response.message);
    %       end

    properties (SetAccess = private)
        %IP IPv4 address or host name of the optimizer server.
        ip

        %PORT TCP port of the optimizer server.
        port

        %TIMEOUT Connect and read timeout in seconds.
        timeout

        %MAXRESPONSEBYTES Safety limit for incoming payload size.
        maxResponseBytes
    end

    methods
        function obj = OpEnTcpOptimizer(arg1, arg2, varargin)
            %OPENTCPOPTIMIZER Construct a TCP client for a generated optimizer.
            %
            %   OBJ = OPENTCPOPTIMIZER(PORT) uses 127.0.0.1.
            %   OBJ = OPENTCPOPTIMIZER(PORT, IP) uses the provided IP.
            %   OBJ = OPENTCPOPTIMIZER(IP, PORT) is also supported.
            %
            %   Name-value pairs:
            %       'Timeout'          Socket timeout in seconds (default 10)
            %       'MaxResponseBytes' Maximum response size in bytes
            %                          (default 1048576)

            if nargin < 1
                error('OpEnTcpOptimizer:NotEnoughInputs', ...
                    'You must provide at least a TCP port.');
            end

            if nargin < 2
                arg2 = [];
            end

            [port, ip] = OpEnTcpOptimizer.normalizeEndpointArguments(arg1, arg2);

            parser = inputParser();
            parser.FunctionName = 'OpEnTcpOptimizer';
            addRequired(parser, 'port', @OpEnTcpOptimizer.isValidPort);
            addRequired(parser, 'ip', @OpEnTcpOptimizer.isTextScalar);
            addParameter(parser, 'Timeout', 10, @OpEnTcpOptimizer.isValidTimeout);
            addParameter(parser, 'MaxResponseBytes', 1048576, @OpEnTcpOptimizer.isValidMaxResponseBytes);
            parse(parser, port, ip, varargin{:});

            obj.port = double(parser.Results.port);
            obj.ip = OpEnTcpOptimizer.textToChar(parser.Results.ip);
            obj.timeout = double(parser.Results.Timeout);
            obj.maxResponseBytes = double(parser.Results.MaxResponseBytes);
        end

        function response = ping(obj)
            %PING Check whether the optimizer server is reachable.
            %   RESPONSE = PING(OBJ) sends {"Ping":1} and returns the
            %   decoded JSON response, typically a struct with field "Pong".
            response = obj.sendRequest('{"Ping":1}', true);
        end

        function kill(obj)
            %KILL Ask the optimizer server to stop gracefully.
            %   KILL(OBJ) sends {"Kill":1}. The server closes the
            %   connection without returning a JSON payload.
            obj.sendRequest('{"Kill":1}', false);
        end

        function response = solve(obj, parameter, varargin)
            %SOLVE Run the optimizer for the given parameter vector.
            %   RESPONSE = SOLVE(OBJ, PARAMETER) sends PARAMETER to the
            %   optimizer and returns a struct with field RESPONSE.ok.
            %
            %   RESPONSE = SOLVE(..., 'InitialGuess', U0,
            %   'InitialLagrangeMultipliers', Y0, 'InitialPenalty', C0)
            %   mirrors the TCP options supported by the generated server.
            %
            %   On success, RESPONSE contains the solver fields returned by
            %   the server, plus the aliases:
            %       ok = true
            %       raw = original decoded JSON response
            %       f1_infeasibility = delta_y_norm_over_c
            %
            %   On failure, RESPONSE contains:
            %       ok = false
            %       raw = original decoded JSON response
            %       code, message

            parser = inputParser();
            parser.FunctionName = 'OpEnTcpOptimizer.solve';
            addRequired(parser, 'parameter', @OpEnTcpOptimizer.isVectorNumeric);
            addParameter(parser, 'InitialGuess', [], @OpEnTcpOptimizer.isOptionalVectorNumeric);
            addParameter(parser, 'InitialLagrangeMultipliers', [], @OpEnTcpOptimizer.isOptionalVectorNumeric);
            addParameter(parser, 'InitialPenalty', [], @OpEnTcpOptimizer.isOptionalScalarNumeric);
            parse(parser, parameter, varargin{:});

            request = struct();
            request.Run = struct();
            request.Run.parameter = OpEnTcpOptimizer.toRowVector(parser.Results.parameter, 'parameter');

            if ~isempty(parser.Results.InitialGuess)
                request.Run.initial_guess = OpEnTcpOptimizer.toRowVector( ...
                    parser.Results.InitialGuess, 'InitialGuess');
            end

            if ~isempty(parser.Results.InitialLagrangeMultipliers)
                request.Run.initial_lagrange_multipliers = OpEnTcpOptimizer.toRowVector( ...
                    parser.Results.InitialLagrangeMultipliers, 'InitialLagrangeMultipliers');
            end

            if ~isempty(parser.Results.InitialPenalty)
                request.Run.initial_penalty = double(parser.Results.InitialPenalty);
            end

            rawResponse = obj.sendRequest(jsonencode(request), true);
            response = OpEnTcpOptimizer.normalizeSolverResponse(rawResponse);
        end

        function response = call(obj, parameter, varargin)
            %CALL Alias for SOLVE to match the Python TCP interface.
            response = obj.solve(parameter, varargin{:});
        end

        function response = consume(obj, parameter, varargin)
            %CONSUME Alias for SOLVE to ease migration from older MATLAB code.
            response = obj.solve(parameter, varargin{:});
        end
    end

    methods (Access = private)
        function response = sendRequest(obj, requestText, expectReply)
            %SENDREQUEST Send a JSON request and optionally decode a JSON reply.
            %
            %   The generated Rust server reads until the client closes its
            %   write side. We therefore use Java sockets so we can call
            %   shutdownOutput() after transmitting the JSON payload.

            socket = [];
            cleanup = [];

            try
                socket = java.net.Socket();
                timeoutMs = max(1, round(1000 * obj.timeout));
                socket.connect(java.net.InetSocketAddress(obj.ip, obj.port), timeoutMs);
                socket.setSoTimeout(timeoutMs);
                cleanup = onCleanup(@() OpEnTcpOptimizer.closeSocketQuietly(socket));

                outputStream = socket.getOutputStream();
                requestBytes = int8(unicode2native(char(requestText), 'UTF-8'));
                outputStream.write(requestBytes);
                outputStream.flush();
                socket.shutdownOutput();

                if ~expectReply
                    return;
                end

                inputStream = socket.getInputStream();
                responseBytes = obj.readFully(inputStream);
                if isempty(responseBytes)
                    error('OpEnTcpOptimizer:EmptyResponse', ...
                        'The optimizer server closed the connection without sending a response.');
                end

                responseText = native2unicode(responseBytes, 'UTF-8');
                response = jsondecode(responseText);
            catch err
                % Ensure the socket is closed before rethrowing transport errors.
                clear cleanup;
                OpEnTcpOptimizer.closeSocketQuietly(socket);
                rethrow(err);
            end

            clear cleanup;
        end

        function bytes = readFully(obj, inputStream)
            %READFULLY Read the complete server reply until EOF.
            %
            %   The server sends a single JSON document per connection and
            %   closes the connection afterwards, so EOF marks the end of
            %   the response payload.

            byteStream = java.io.ByteArrayOutputStream();

            while true
                nextByte = inputStream.read();
                if nextByte == -1
                    break;
                end

                byteStream.write(nextByte);
                if byteStream.size() > obj.maxResponseBytes
                    error('OpEnTcpOptimizer:ResponseTooLarge', ...
                        'The optimizer response exceeded %d bytes.', obj.maxResponseBytes);
                end
            end

            rawBytes = uint8(mod(double(byteStream.toByteArray()), 256));
            bytes = reshape(rawBytes, 1, []);
        end
    end

    methods (Static, Access = private)
        function response = normalizeSolverResponse(rawResponse)
            %NORMALIZESOLVERRESPONSE Add MATLAB-friendly fields to server data.

            response = rawResponse;
            response.raw = rawResponse;

            if isfield(rawResponse, 'type') && strcmp(rawResponse.type, 'Error')
                response.ok = false;
                return;
            end

            response.ok = true;
            if isfield(rawResponse, 'delta_y_norm_over_c')
                response.f1_infeasibility = rawResponse.delta_y_norm_over_c;
            end
        end

        function vector = toRowVector(value, argumentName)
            %TOROWVECTOR Validate a numeric vector and serialize it as a row.
            if ~OpEnTcpOptimizer.isVectorNumeric(value)
                error('OpEnTcpOptimizer:InvalidVector', ...
                    '%s must be a numeric vector.', argumentName);
            end

            vector = reshape(double(value), 1, []);
        end

        function closeSocketQuietly(socket)
            %CLOSESOCKETQUIETLY Best-effort socket close for cleanup paths.
            if isempty(socket)
                return;
            end

            try
                socket.close();
            catch
                % Ignore close errors during cleanup.
            end
        end

        function tf = isValidPort(value)
            %ISVALIDPORT Validate a TCP port number.
            tf = isnumeric(value) && isscalar(value) && isfinite(value) ...
                && value == fix(value) && value >= 1 && value <= 65535;
        end

        function tf = isValidTimeout(value)
            %ISVALIDTIMEOUT Validate a positive timeout.
            tf = isnumeric(value) && isscalar(value) && isfinite(value) && value > 0;
        end

        function tf = isValidMaxResponseBytes(value)
            %ISVALIDMAXRESPONSEBYTES Validate the maximum response size.
            tf = isnumeric(value) && isscalar(value) && isfinite(value) ...
                && value == fix(value) && value > 0;
        end

        function tf = isTextScalar(value)
            %ISTEXTSCALAR True for character vectors and string scalars.
            tf = ischar(value) || (isstring(value) && isscalar(value));
        end

        function [port, ip] = normalizeEndpointArguments(arg1, arg2)
            %NORMALIZEENDPOINTARGUMENTS Support both (port, ip) and (ip, port).
            defaultIp = '127.0.0.1';

            if isempty(arg2)
                port = arg1;
                ip = defaultIp;
                return;
            end

            if OpEnTcpOptimizer.isValidPort(arg1) && OpEnTcpOptimizer.isTextScalar(arg2)
                port = arg1;
                ip = arg2;
                return;
            end

            if OpEnTcpOptimizer.isTextScalar(arg1) && OpEnTcpOptimizer.isValidPort(arg2)
                port = arg2;
                ip = arg1;
                return;
            end

            error('OpEnTcpOptimizer:InvalidEndpoint', ...
                ['Specify the endpoint as (port), (port, ip), or (ip, port), ' ...
                 'where port is an integer in [1, 65535].']);
        end

        function value = textToChar(value)
            %TEXTTOCHAR Convert a MATLAB text scalar to a character vector.
            if isstring(value)
                value = char(value);
            end
        end

        function tf = isVectorNumeric(value)
            %ISVECTORNUMERIC True for finite numeric vectors.
            tf = isnumeric(value) && isvector(value) && all(isfinite(value));
        end

        function tf = isOptionalVectorNumeric(value)
            %ISOPTIONALVECTORNUMERIC True for [] or a numeric vector.
            tf = isempty(value) || OpEnTcpOptimizer.isVectorNumeric(value);
        end

        function tf = isOptionalScalarNumeric(value)
            %ISOPTIONALSCALARNUMERIC True for [] or a finite numeric scalar.
            tf = isempty(value) || (isnumeric(value) && isscalar(value) && isfinite(value));
        end
    end
end
