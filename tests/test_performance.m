function summary = test_performance()
%TEST_PERFORMANCE Run repeated end-to-end simulations and record wall time.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

config = params();
config.enableVisualization = false;
config.visualizerVisible = false;
config.dt = 1.0;
config.totalTime = 260.0;
config.dynamicObstacleSchedule = struct( ...
    'startTime', 0.0, ...
    'endTime', 3.0, ...
    'positions', [1, 24]);

runCount = 3;
runtimes = zeros(runCount, 1);
completionTimes = zeros(runCount, 1);

for i = 1:runCount
    simulation = sim.Simulation.fromDefaults(config);
    timerId = tic;
    results = simulation.run();
    runtimes(i) = toc(timerId);
    completionTimes(i) = results.currentTime;

    assert(results.completedTaskCount == 5, ...
        'Performance run should still complete all tasks.');
    assert(runtimes(i) < config.totalTime, ...
        'Wall-clock runtime should remain below the simulated scenario horizon.');
end

summary = struct( ...
    'runs', runCount, ...
    'minRuntime', min(runtimes), ...
    'meanRuntime', mean(runtimes), ...
    'maxRuntime', max(runtimes), ...
    'scenarioCompletionTimes', completionTimes);

disp('Performance summary:');
disp(summary);
disp('test_performance passed');
end
