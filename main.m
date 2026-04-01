%MAIN Project entry point for the multi-AGV warehouse simulation.
%   Loads default parameters, builds the default simulation objects, runs
%   the scenario once, and prints the final completion summary.

config = params();
simulation = sim.Simulation.fromDefaults(config);
results = simulation.run();

fprintf('Simulation finished at %.1f s with %d completed tasks.\n', ...
    results.currentTime, results.completedTaskCount);
