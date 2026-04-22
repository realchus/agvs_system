function config = params()
%PARAMS Return default simulation configuration values.
% Output:
%   config - Struct containing the default headless/visual simulation
%            options, conflict wait timeout, and optional dynamic
%            obstacle schedule used by tests or demo runs.

config = struct();
config.dt = 0.1;
config.totalTime = 4200.0;
config.enableHeadlessMode = true;
config.enableVisualization = ~config.enableHeadlessMode;
config.visualizerVisible = ~config.enableHeadlessMode;
config.returnToParkingWhenIdle = true;
config.waitTimeout = 5.0;
config.dynamicObstacleSchedule = repmat(struct( ...
    'startTime', 0.0, ...
    'endTime', 0.0, ...
    'positions', zeros(0, 2)), 0, 1);
end
