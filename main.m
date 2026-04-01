config = params();
simulation = sim.Simulation.fromDefaults(config);
results = simulation.run();

fprintf('Simulation finished at %.1f s with %d completed tasks.\n', ...
    results.currentTime, results.completedTaskCount);
