function config = params()
%PARAMS Return default simulation configuration values.
% Output:
%   config - Struct containing the default headless/visual simulation
%            options, conflict wait timeout, and optional dynamic
%            obstacle schedule used by tests or demo runs.

projectRoot = fileparts(mfilename('fullpath'));

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
config.agvPoolFile = fullfile(projectRoot, 'data', 'agv_pool.json');
config.taskListFile = fullfile(projectRoot, 'data', 'task_list.json');
end
