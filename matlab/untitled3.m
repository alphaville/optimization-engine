lepath = "/Users/pantelis/Documents/Development/optimization-engine/open-codegen/.python_test_build_ocp/ocp_single_tcp/optimizer_manifest.json";
client = OpEnTcpOptimizer(3391, 'ManifestPath', lepath );

%%
clc
response = client.solve(x0=[1.2, -2.0], ...
    xref=[0.5, 2], ...
    InitialPenalty=1)
us = [response.inputs{:}];
plot(us, '-o'); grid on;

%%

client = OpEnTcpOptimizer(17880);
client.solve([1., 2.], 'InitialGuess', [1., 3., 0., 0., 5.])