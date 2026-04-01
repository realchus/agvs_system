function config = params()
%PARAMS Return default simulation configuration values.

config = struct();
config.dt = 0.1;
config.totalTime = 180.0;
config.enableVisualization = usejava('desktop');
config.visualizerVisible = config.enableVisualization;
config.waitTimeout = 5.0;
config.dynamicObstacleSchedule = repmat(struct( ...
    'startTime', 0.0, ...
    'endTime', 0.0, ...
    'positions', zeros(0, 2)), 0, 1);
end
