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
    %   CLIENT = OPENTCPOPTIMIZER(..., 'ManifestPath', MANIFESTPATH) loads
    %   an OCP optimizer manifest created by Python's ``ocp`` module. Once
    %   a manifest is loaded, the client also supports named-parameter
    %   calls such as:
    %
    %       response = client.solve(x0=[1; 0], xref=[0; 0]);
    %
    %   Name-value pairs:
    %       'ManifestPath'     Path to optimizer_manifest.json
    %       'Timeout'          Socket timeout in seconds (default 10)
    %       'MaxResponseBytes' Maximum response size in bytes
    %                          (default 1048576)
    %
    %   If only a manifest path is provided, the constructor attempts to
    %   read ``optimizer.yml`` next to the manifest and use its TCP/IP
    %   endpoint automatically.
    %
    %   This interface is intended for optimizers generated in Python with
    %   the TCP interface enabled. The optimizer server must already be
    %   running; this class only communicates with it.
    %
    %   Examples:
    %       client = OpEnTcpOptimizer(3301);
    %       response = client.solve([2.0, 10.0]);
    %
    %       client = OpEnTcpOptimizer('ManifestPath', 'optimizer_manifest.json');
    %       response = client.solve(x0=[1.0, -1.0], xref=[0.0, 0.0]);

    properties (SetAccess = private)
        %IP IPv4 address or host name of the optimizer server.
        ip

        %PORT TCP port of the optimizer server.
        port

        %TIMEOUT Connect and read timeout in seconds.
        timeout

        %MAXRESPONSEBYTES Safety limit for incoming payload size.
        maxResponseBytes

        %MANIFESTPATH Absolute path to an optional OCP manifest.
        manifestPath

        %MANIFEST Decoded OCP manifest data.
        manifest
    end

    methods
        function obj = OpEnTcpOptimizer(varargin)
            %OPENTCPOPTIMIZER Construct a TCP client for a generated optimizer.
            %
            %   Supported call patterns:
            %       OpEnTcpOptimizer(port)
            %       OpEnTcpOptimizer(port, ip)
            %       OpEnTcpOptimizer(ip, port)
            %       OpEnTcpOptimizer(..., 'ManifestPath', path)
            %       OpEnTcpOptimizer('ManifestPath', path)

            obj.manifestPath = '';
            obj.manifest = [];

            [endpointArgs, options] = OpEnTcpOptimizer.parseConstructorInputs(varargin);

            if ~isempty(options.ManifestPath)
                obj.loadManifest(options.ManifestPath);
            end

            [port, ip] = obj.resolveEndpoint(endpointArgs);

            obj.port = double(port);
            obj.ip = OpEnTcpOptimizer.textToChar(ip);
            obj.timeout = double(options.Timeout);
            obj.maxResponseBytes = double(options.MaxResponseBytes);
        end

        function obj = loadManifest(obj, manifestPath)
            %LOADMANIFEST Load an OCP optimizer manifest.
            %   OBJ = LOADMANIFEST(OBJ, MANIFESTPATH) loads an
            %   ``optimizer_manifest.json`` file created by the Python OCP
            %   module. After loading the manifest, the client accepts
            %   named-parameter calls such as ``solve(x0=..., xref=...)``.

            if ~OpEnTcpOptimizer.isTextScalar(manifestPath)
                error('OpEnTcpOptimizer:InvalidManifestPath', ...
                    'ManifestPath must be a character vector or string scalar.');
            end
            manifestPath = OpEnTcpOptimizer.textToChar(manifestPath);
            manifestPath = OpEnTcpOptimizer.validateManifestPath(manifestPath);
            manifestText = fileread(manifestPath);
            manifestData = jsondecode(manifestText);

            if ~isstruct(manifestData)
                error('OpEnTcpOptimizer:InvalidManifest', ...
                    'The manifest must decode to a MATLAB struct.');
            end

            if ~isfield(manifestData, 'parameters')
                error('OpEnTcpOptimizer:InvalidManifest', ...
                    'The manifest does not contain a "parameters" field.');
            end

            OpEnTcpOptimizer.validateManifestParameters(manifestData.parameters);

            obj.manifestPath = manifestPath;
            obj.manifest = manifestData;
        end

        function tf = hasManifest(obj)
            %HASMANIFEST True if an OCP manifest has been loaded.
            tf = ~isempty(obj.manifest);
        end

        function names = parameterNames(obj)
            %PARAMETERNAMES Return the ordered OCP parameter names.
            if ~obj.hasManifest()
                names = {};
                return;
            end

            definitions = OpEnTcpOptimizer.manifestParametersAsCell(obj.manifest.parameters);
            names = cell(size(definitions));
            for i = 1:numel(definitions)
                names{i} = definitions{i}.name;
            end
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

        function response = solve(obj, varargin)
            %SOLVE Run a parametric or OCP optimizer over TCP.
            %   RESPONSE = SOLVE(OBJ, P) sends the flat parameter vector P
            %   to a standard parametric optimizer.
            %
            %   RESPONSE = SOLVE(OBJ, x0=..., xref=..., ...) packs the named
            %   parameter blocks declared in the loaded OCP manifest and
            %   sends the resulting flat parameter vector to the solver.
            %
            %   In both modes, the optional solver warm-start arguments are:
            %       InitialGuess
            %       InitialLagrangeMultipliers
            %       InitialPenalty
            %
            %   On success, RESPONSE contains the solver fields returned by
            %   the server, plus:
            %       ok = true
            %       raw = original decoded JSON response
            %       f1_infeasibility = delta_y_norm_over_c
            %
            %   For OCP solves, RESPONSE also contains:
            %       packed_parameter = flat parameter vector sent to server
            %       inputs = stage-wise control inputs, when available
            %       states = state trajectory for multiple shooting OCPs

            [parameterVector, solverOptions, solveMode] = obj.prepareSolveInputs(varargin);
            rawResponse = obj.runSolveRequest(parameterVector, solverOptions);
            response = OpEnTcpOptimizer.normalizeSolverResponse(rawResponse);

            if strcmp(solveMode, 'ocp')
                response.packed_parameter = parameterVector;
                if response.ok
                    response = obj.enrichOcpResponse(response, parameterVector);
                end
            end
        end

        function response = call(obj, varargin)
            %CALL Alias for SOLVE to match the Python TCP interface.
            response = obj.solve(varargin{:});
        end

        function response = consume(obj, varargin)
            %CONSUME Alias for SOLVE to ease migration from older MATLAB code.
            response = obj.solve(varargin{:});
        end
    end

    methods (Access = private)
        function [parameterVector, solverOptions, solveMode] = prepareSolveInputs(obj, inputArgs)
            %PREPARESOLVEINPUTS Parse parametric or OCP solve inputs.

            if isempty(inputArgs)
                error('OpEnTcpOptimizer:MissingSolveArguments', ...
                    'Provide either a flat parameter vector or named OCP parameters.');
            end

            if OpEnTcpOptimizer.isVectorNumeric(inputArgs{1})
                solveMode = 'parametric';
                parameterVector = OpEnTcpOptimizer.toRowVector(inputArgs{1}, 'parameter');
                solverOptions = OpEnTcpOptimizer.parseSolverOptions(inputArgs(2:end));
                return;
            end

            if ~obj.hasManifest()
                error('OpEnTcpOptimizer:ManifestRequired', ...
                    ['Named parameter solves require an OCP manifest. Load one with ' ...
                     'loadManifest(...) or the constructor option ''ManifestPath''.']);
            end

            [parameterVector, solverOptions] = obj.packOcpParameters(inputArgs);
            solveMode = 'ocp';
        end

        function [parameterVector, solverOptions] = packOcpParameters(obj, inputArgs)
            %PACKOCPPARAMETERS Pack named OCP parameters using the manifest.

            pairs = OpEnTcpOptimizer.normalizeNameValuePairs(inputArgs, 'OpEnTcpOptimizer.solve');
            solverOptions = OpEnTcpOptimizer.emptySolverOptions();
            providedValues = containers.Map('KeyType', 'char', 'ValueType', 'any');

            for i = 1:size(pairs, 1)
                name = pairs{i, 1};
                value = pairs{i, 2};
                lowerName = lower(name);

                switch lowerName
                    case 'initialguess'
                        solverOptions.InitialGuess = value;
                    case 'initiallagrangemultipliers'
                        solverOptions.InitialLagrangeMultipliers = value;
                    case 'initialpenalty'
                        solverOptions.InitialPenalty = value;
                    otherwise
                        if isKey(providedValues, name)
                            error('OpEnTcpOptimizer:DuplicateParameter', ...
                                'Parameter "%s" was provided more than once.', name);
                        end
                        providedValues(name) = value;
                end
            end

            solverOptions = OpEnTcpOptimizer.validateSolverOptions(solverOptions);

            definitions = OpEnTcpOptimizer.manifestParametersAsCell(obj.manifest.parameters);
            parameterVector = [];
            missing = {};

            for i = 1:numel(definitions)
                definition = definitions{i};
                if isKey(providedValues, definition.name)
                    value = providedValues(definition.name);
                    remove(providedValues, definition.name);
                else
                    value = definition.default;
                end

                if isempty(value)
                    missing{end + 1} = definition.name; %#ok<AGROW>
                    continue;
                end

                parameterVector = [parameterVector, ... %#ok<AGROW>
                    OpEnTcpOptimizer.normalizeParameterBlock( ...
                        value, definition.size, definition.name)];
            end

            if ~isempty(missing)
                error('OpEnTcpOptimizer:MissingOcpParameters', ...
                    'Missing values for parameters: %s.', strjoin(missing, ', '));
            end

            remainingNames = sort(keys(providedValues));
            if ~isempty(remainingNames)
                error('OpEnTcpOptimizer:UnknownOcpParameters', ...
                    'Unknown OCP parameter(s): %s.', strjoin(remainingNames, ', '));
            end
        end

        function response = enrichOcpResponse(obj, response, packedParameters)
            %ENRICHOCPRESPONSE Add OCP-oriented views to a successful solve.

            response.inputs = obj.extractOcpInputs(response.solution);

            if strcmp(obj.manifest.shooting, 'multiple')
                response.states = obj.extractMultipleShootingStates( ...
                    response.solution, packedParameters);
            end
        end

        function inputs = extractOcpInputs(obj, flatSolution)
            %EXTRACTOCPINPUTS Extract stage-wise input blocks from the solution.

            flatSolution = OpEnTcpOptimizer.toRowVector(flatSolution, 'solution');

            if strcmp(obj.manifest.shooting, 'single')
                nu = double(obj.manifest.nu);
                horizon = double(obj.manifest.horizon);
                inputs = cell(1, horizon);

                for stageIdx = 1:horizon
                    startIdx = (stageIdx - 1) * nu + 1;
                    stopIdx = stageIdx * nu;
                    inputs{stageIdx} = flatSolution(startIdx:stopIdx);
                end
                return;
            end

            sliceMatrix = obj.manifest.input_slices;
            inputs = cell(1, size(sliceMatrix, 1));
            for i = 1:size(sliceMatrix, 1)
                startIdx = sliceMatrix(i, 1) + 1;
                stopIdx = sliceMatrix(i, 2);
                inputs{i} = flatSolution(startIdx:stopIdx);
            end
        end

        function states = extractMultipleShootingStates(obj, flatSolution, packedParameters)
            %EXTRACTMULTIPLESHOOTINGSTATES Extract the state trajectory.
            %
            %   For multiple shooting OCPs the manifest contains the state
            %   slices directly, so no extra CasADi dependency is needed in
            %   MATLAB to reconstruct the state trajectory.

            flatSolution = OpEnTcpOptimizer.toRowVector(flatSolution, 'solution');
            packedParameters = OpEnTcpOptimizer.toRowVector(packedParameters, 'packedParameters');

            stateSlices = obj.manifest.state_slices;
            states = cell(1, size(stateSlices, 1) + 1);
            states{1} = obj.extractParameterByName(packedParameters, 'x0');

            for i = 1:size(stateSlices, 1)
                startIdx = stateSlices(i, 1) + 1;
                stopIdx = stateSlices(i, 2);
                states{i + 1} = flatSolution(startIdx:stopIdx);
            end
        end

        function value = extractParameterByName(obj, packedParameters, parameterName)
            %EXTRACTPARAMETERBYNAME Extract one named parameter block.

            definitions = OpEnTcpOptimizer.manifestParametersAsCell(obj.manifest.parameters);
            offset = 0;

            for i = 1:numel(definitions)
                definition = definitions{i};
                nextOffset = offset + definition.size;
                if strcmp(definition.name, parameterName)
                    value = packedParameters(offset + 1:nextOffset);
                    return;
                end
                offset = nextOffset;
            end

            error('OpEnTcpOptimizer:MissingManifestParameter', ...
                'The manifest does not define a parameter named "%s".', parameterName);
        end

        function rawResponse = runSolveRequest(obj, parameterVector, solverOptions)
            %RUNSOLVEREQUEST Serialize and send a solver execution request.

            request = struct();
            request.Run = struct();
            request.Run.parameter = OpEnTcpOptimizer.toRowVector(parameterVector, 'parameter');

            if ~isempty(solverOptions.InitialGuess)
                request.Run.initial_guess = OpEnTcpOptimizer.toRowVector( ...
                    solverOptions.InitialGuess, 'InitialGuess');
            end

            if ~isempty(solverOptions.InitialLagrangeMultipliers)
                request.Run.initial_lagrange_multipliers = OpEnTcpOptimizer.toRowVector( ...
                    solverOptions.InitialLagrangeMultipliers, 'InitialLagrangeMultipliers');
            end

            if ~isempty(solverOptions.InitialPenalty)
                request.Run.initial_penalty = double(solverOptions.InitialPenalty);
            end

            rawResponse = obj.sendRequest(jsonencode(request), true);
        end

        function [port, ip] = resolveEndpoint(obj, endpointArgs)
            %RESOLVEENDPOINT Resolve the TCP endpoint from inputs or manifest.

            if isempty(endpointArgs)
                if ~obj.hasManifest()
                    error('OpEnTcpOptimizer:MissingEndpoint', ...
                        'Provide a TCP endpoint or a manifest with a matching optimizer.yml file.');
                end

                tcpDefaults = OpEnTcpOptimizer.readTcpDefaultsFromManifest(obj.manifestPath);
                if isempty(tcpDefaults)
                    error('OpEnTcpOptimizer:MissingEndpoint', ...
                        ['No TCP endpoint was provided and no TCP settings could be read from ' ...
                         'optimizer.yml next to the manifest.']);
                end

                port = tcpDefaults.port;
                ip = tcpDefaults.ip;
                return;
            end

            if numel(endpointArgs) == 1
                if ~OpEnTcpOptimizer.isValidPort(endpointArgs{1})
                    error('OpEnTcpOptimizer:InvalidEndpoint', ...
                        'A single endpoint argument must be a TCP port.');
                end
                port = endpointArgs{1};
                ip = '127.0.0.1';
                return;
            end

            if numel(endpointArgs) == 2
                [port, ip] = OpEnTcpOptimizer.normalizeEndpointArguments( ...
                    endpointArgs{1}, endpointArgs{2});
                return;
            end

            error('OpEnTcpOptimizer:InvalidEndpoint', ...
                'Specify the endpoint as (port), (port, ip), or (ip, port).');
        end

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
        function [endpointArgs, options] = parseConstructorInputs(inputArgs)
            %PARSECONSTRUCTORINPUTS Split constructor endpoint and options.

            endpointArgs = {};
            options = struct( ...
                'ManifestPath', '', ...
                'Timeout', 10, ...
                'MaxResponseBytes', 1048576);

            idx = 1;
            while idx <= numel(inputArgs)
                token = inputArgs{idx};
                if OpEnTcpOptimizer.isRecognizedConstructorOption(token)
                    if idx == numel(inputArgs)
                        error('OpEnTcpOptimizer:InvalidConstructorInput', ...
                            'Missing value for option "%s".', OpEnTcpOptimizer.textToChar(token));
                    end

                    name = lower(OpEnTcpOptimizer.textToChar(token));
                    value = inputArgs{idx + 1};
                    switch name
                        case 'manifestpath'
                            if ~OpEnTcpOptimizer.isTextScalar(value)
                                error('OpEnTcpOptimizer:InvalidManifestPath', ...
                                    'ManifestPath must be a character vector or string scalar.');
                            end
                            options.ManifestPath = OpEnTcpOptimizer.textToChar(value);
                        case 'timeout'
                            options.Timeout = value;
                        case 'maxresponsebytes'
                            options.MaxResponseBytes = value;
                    end
                    idx = idx + 2;
                else
                    endpointArgs{end + 1} = token; %#ok<AGROW>
                    idx = idx + 1;
                end
            end

            if numel(endpointArgs) == 1 && OpEnTcpOptimizer.isManifestPathToken(endpointArgs{1})
                options.ManifestPath = OpEnTcpOptimizer.textToChar(endpointArgs{1});
                endpointArgs = {};
            end

            if ~(OpEnTcpOptimizer.isValidTimeout(options.Timeout))
                error('OpEnTcpOptimizer:InvalidTimeout', ...
                    'Timeout must be a positive scalar.');
            end

            if ~(OpEnTcpOptimizer.isValidMaxResponseBytes(options.MaxResponseBytes))
                error('OpEnTcpOptimizer:InvalidMaxResponseBytes', ...
                    'MaxResponseBytes must be a positive integer.');
            end
        end

        function tf = isRecognizedConstructorOption(token)
            %ISRECOGNIZEDCONSTRUCTOROPTION True for constructor option names.
            tf = OpEnTcpOptimizer.isTextScalar(token) && any(strcmpi( ...
                OpEnTcpOptimizer.textToChar(token), {'ManifestPath', 'Timeout', 'MaxResponseBytes'}));
        end

        function tf = isManifestPathToken(token)
            %ISMANIFESTPATHTOKEN Heuristic for a positional manifest path.
            if ~OpEnTcpOptimizer.isTextScalar(token)
                tf = false;
                return;
            end

            token = OpEnTcpOptimizer.textToChar(token);
            [~, ~, ext] = fileparts(token);
            tf = strcmpi(ext, '.json') && isfile(token);
        end

        function manifestPath = validateManifestPath(manifestPath)
            %VALIDATEMANIFESTPATH Validate and absolutize a manifest path.
            if ~isfile(manifestPath)
                error('OpEnTcpOptimizer:ManifestNotFound', ...
                    'Manifest file not found: %s', manifestPath);
            end

            [folder, name, ext] = fileparts(manifestPath);
            if ~strcmpi(ext, '.json')
                error('OpEnTcpOptimizer:InvalidManifestPath', ...
                    'The manifest path must point to a JSON file.');
            end

            manifestPath = fullfile(folder, [name, ext]);
        end

        function validateManifestParameters(parameters)
            %VALIDATEMANIFESTPARAMETERS Validate manifest parameter entries.
            definitions = OpEnTcpOptimizer.manifestParametersAsCell(parameters);
            for i = 1:numel(definitions)
                definition = definitions{i};
                if ~isfield(definition, 'name') || ~isfield(definition, 'size')
                    error('OpEnTcpOptimizer:InvalidManifest', ...
                        'Each manifest parameter needs "name" and "size" fields.');
                end
                if ~ischar(definition.name) && ~isstring(definition.name)
                    error('OpEnTcpOptimizer:InvalidManifest', ...
                        'Manifest parameter names must be text values.');
                end
                if ~OpEnTcpOptimizer.isValidPositiveInteger(definition.size)
                    error('OpEnTcpOptimizer:InvalidManifest', ...
                        'Manifest parameter sizes must be positive integers.');
                end
            end
        end

        function defaults = readTcpDefaultsFromManifest(manifestPath)
            %READTCPDEFAULTSFROMMANIFEST Read TCP defaults from optimizer.yml.
            defaults = [];
            optimizerDir = fileparts(manifestPath);
            yamlPath = fullfile(optimizerDir, 'optimizer.yml');

            if ~isfile(yamlPath)
                return;
            end

            defaults = OpEnTcpOptimizer.parseOptimizerYaml(yamlPath);
        end

        function defaults = parseOptimizerYaml(yamlPath)
            %PARSEOPTIMIZERYAML Read the tcp.ip and tcp.port fields.
            defaults = [];
            yamlText = fileread(yamlPath);
            lines = regexp(yamlText, '\r\n|\n|\r', 'split');

            inTcpBlock = false;
            ip = '';
            port = [];

            for i = 1:numel(lines)
                line = lines{i};
                trimmed = strtrim(line);

                if isempty(trimmed)
                    continue;
                end

                if strcmp(trimmed, 'tcp:')
                    inTcpBlock = true;
                    continue;
                end

                if inTcpBlock && ~isempty(line) && ~isspace(line(1))
                    break;
                end

                if inTcpBlock
                    if startsWith(trimmed, 'ip:')
                        ip = strtrim(extractAfter(trimmed, 3));
                    elseif startsWith(trimmed, 'port:')
                        portText = strtrim(extractAfter(trimmed, 5));
                        port = str2double(portText);
                    end
                end
            end

            if ~isempty(ip) && OpEnTcpOptimizer.isValidPort(port)
                defaults = struct('ip', ip, 'port', port);
            end
        end

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

        function solverOptions = parseSolverOptions(inputArgs)
            %PARSESOLVEROPTIONS Parse warm-start related name-value pairs.
            pairs = OpEnTcpOptimizer.normalizeNameValuePairs(inputArgs, 'OpEnTcpOptimizer.solve');
            solverOptions = OpEnTcpOptimizer.emptySolverOptions();

            for i = 1:size(pairs, 1)
                name = lower(pairs{i, 1});
                value = pairs{i, 2};

                switch name
                    case 'initialguess'
                        solverOptions.InitialGuess = value;
                    case 'initiallagrangemultipliers'
                        solverOptions.InitialLagrangeMultipliers = value;
                    case 'initialpenalty'
                        solverOptions.InitialPenalty = value;
                    otherwise
                        error('OpEnTcpOptimizer:UnknownSolveOption', ...
                            'Unknown solve option "%s".', pairs{i, 1});
                end
            end

            solverOptions = OpEnTcpOptimizer.validateSolverOptions(solverOptions);
        end

        function solverOptions = validateSolverOptions(solverOptions)
            %VALIDATESOLVEROPTIONS Validate optional warm-start inputs.
            if ~OpEnTcpOptimizer.isOptionalVectorNumeric(solverOptions.InitialGuess)
                error('OpEnTcpOptimizer:InvalidInitialGuess', ...
                    'InitialGuess must be a numeric vector or [].');
            end

            if ~OpEnTcpOptimizer.isOptionalVectorNumeric(solverOptions.InitialLagrangeMultipliers)
                error('OpEnTcpOptimizer:InvalidInitialLagrangeMultipliers', ...
                    'InitialLagrangeMultipliers must be a numeric vector or [].');
            end

            if ~OpEnTcpOptimizer.isOptionalScalarNumeric(solverOptions.InitialPenalty)
                error('OpEnTcpOptimizer:InvalidInitialPenalty', ...
                    'InitialPenalty must be a numeric scalar or [].');
            end
        end

        function solverOptions = emptySolverOptions()
            %EMPTYSOLVEROPTIONS Return the default solve option bundle.
            solverOptions = struct( ...
                'InitialGuess', [], ...
                'InitialLagrangeMultipliers', [], ...
                'InitialPenalty', []);
        end

        function pairs = normalizeNameValuePairs(inputArgs, functionName)
            %NORMALIZENAMEVALUEPAIRS Validate and normalize name-value pairs.
            if isempty(inputArgs)
                pairs = cell(0, 2);
                return;
            end

            if mod(numel(inputArgs), 2) ~= 0
                error('OpEnTcpOptimizer:InvalidNameValueInput', ...
                    '%s expects name-value arguments in pairs.', functionName);
            end

            pairs = cell(numel(inputArgs) / 2, 2);
            pairIdx = 1;
            for i = 1:2:numel(inputArgs)
                name = inputArgs{i};
                if ~OpEnTcpOptimizer.isTextScalar(name)
                    error('OpEnTcpOptimizer:InvalidNameValueInput', ...
                        'Expected a text parameter name at argument position %d.', i);
                end

                pairs{pairIdx, 1} = OpEnTcpOptimizer.textToChar(name);
                pairs{pairIdx, 2} = inputArgs{i + 1};
                pairIdx = pairIdx + 1;
            end
        end

        function blocks = manifestParametersAsCell(parameters)
            %MANIFESTPARAMETERSASCELL Normalize decoded parameter definitions.
            if isempty(parameters)
                blocks = {};
                return;
            end

            if isstruct(parameters)
                blocks = cell(1, numel(parameters));
                for i = 1:numel(parameters)
                    blocks{i} = parameters(i);
                end
                return;
            end

            error('OpEnTcpOptimizer:InvalidManifest', ...
                'Manifest parameters must decode to a struct array.');
        end

        function vector = normalizeParameterBlock(value, expectedSize, parameterName)
            %NORMALIZEPARAMETERBLOCK Normalize one OCP parameter block.
            if expectedSize == 1 && isnumeric(value) && isscalar(value) && isfinite(value)
                vector = double(value);
                return;
            end

            if ~OpEnTcpOptimizer.isVectorNumeric(value)
                error('OpEnTcpOptimizer:InvalidOcpParameter', ...
                    'Parameter "%s" must be a numeric vector.', parameterName);
            end

            vector = reshape(double(value), 1, []);
            if numel(vector) ~= double(expectedSize)
                error('OpEnTcpOptimizer:InvalidOcpParameterDimension', ...
                    'Parameter "%s" must have length %d.', parameterName, double(expectedSize));
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

        function [port, ip] = normalizeEndpointArguments(arg1, arg2)
            %NORMALIZEENDPOINTARGUMENTS Support both (port, ip) and (ip, port).
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

        function tf = isValidPort(value)
            %ISVALIDPORT Validate a TCP port number.
            tf = OpEnTcpOptimizer.isValidPositiveInteger(value) ...
                && double(value) >= 1 && double(value) <= 65535;
        end

        function tf = isValidTimeout(value)
            %ISVALIDTIMEOUT Validate a positive timeout.
            tf = isnumeric(value) && isscalar(value) && isfinite(value) && value > 0;
        end

        function tf = isValidMaxResponseBytes(value)
            %ISVALIDMAXRESPONSEBYTES Validate the maximum response size.
            tf = OpEnTcpOptimizer.isValidPositiveInteger(value);
        end

        function tf = isValidPositiveInteger(value)
            %ISVALIDPOSITIVEINTEGER Validate a positive integer scalar.
            tf = isnumeric(value) && isscalar(value) && isfinite(value) ...
                && value == fix(value) && value > 0;
        end

        function tf = isTextScalar(value)
            %ISTEXTSCALAR True for character vectors and string scalars.
            tf = ischar(value) || (isstring(value) && isscalar(value));
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
