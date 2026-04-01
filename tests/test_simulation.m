function test_simulation()
%TEST_SIMULATION Validate simulation loop integration and conflict handling.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

testSingleTaskLifecycle();
testConflictResolutionInsideStep();

disp('test_simulation passed');
end

function testSingleTaskLifecycle()
simMap = map.MapClass(zeros(3, 6), map.MapClass.defaultColors());
vehicle = agv.AGVClass(1, [2, 1], 1.0);
taskObj = task.TaskClass(501, [2, 1], ...
    [struct('name', 'Pickup', 'position', [2, 3]); struct('name', 'Dropoff', 'position', [2, 6])], ...
    1, 0, 'pending');

config = struct( ...
    'dt', 1.0, ...
    'totalTime', 20.0, ...
    'enableVisualization', false, ...
    'visualizerVisible', false, ...
    'waitTimeout', 2.0, ...
    'pathLibraryData', repmat(struct(), 0, 1), ...
    'dynamicObstacleSchedule', repmat(struct(), 0, 1));

simulation = sim.Simulation(simMap, vehicle, taskObj, config);
results = simulation.run();

assert(strcmp(taskObj.status, 'completed'), 'Simulation should complete the pending task.');
assert(isempty(vehicle.currentTask), 'AGV should release the task after completion.');
assert(strcmp(vehicle.state, 'idle'), 'AGV should return to idle when the task is done.');
assert(results.completedTaskCount == 1, 'Completed task count should be reported correctly.');
assert(any(strcmp({simulation.eventLog.type}, 'task_assigned')), 'Event log should include task assignment.');
assert(any(strcmp({simulation.eventLog.type}, 'loading_started')), 'Event log should include loading.');
assert(any(strcmp({simulation.eventLog.type}, 'task_completed')), 'Event log should include task completion.');
end

function testConflictResolutionInsideStep()
simMap = map.MapClass(zeros(3, 5), map.MapClass.defaultColors());
manager = timewindow.TimeWindowManager();

highTask = task.TaskClass(601, [2, 1], struct('name', 'GoalA', 'position', [2, 4]), 5, 0, 'executing');
lowTask = task.TaskClass(602, [2, 2], struct('name', 'GoalB', 'position', [2, 5]), 1, 10, 'executing');

agvHigh = agv.AGVClass(1, [2, 1], 1.0);
agvLow = agv.AGVClass(2, [2, 2], 1.0);

highPath = [2, 1; 2, 2; 2, 3; 2, 4];
lowPath = [2, 2; 2, 3; 2, 4; 2, 5];
[highWindows, highConflict] = manager.reservePath(agvHigh.id, highPath, 0.0, agvHigh.speed);
assert(isempty(highConflict), 'Primary reservation should succeed.');

tempManager = timewindow.TimeWindowManager();
[lowWindows, lowConflict] = tempManager.reservePath(agvLow.id, lowPath, 1.0, agvLow.speed);
assert(isempty(lowConflict), 'Secondary reservation template should succeed in isolation.');
manager.timeWindows = [highWindows; lowWindows];

agvHigh.assignTask(highTask);
agvHigh.assignPath(highPath);
agvHigh.setTimeWindows(highWindows);

agvLow.assignTask(lowTask);
agvLow.assignPath(lowPath);
agvLow.setTimeWindows(lowWindows);

schedulerObj = scheduler.SchedulerClass(simMap, [agvHigh; agvLow], [highTask; lowTask], repmat(struct(), 0, 1), manager);
config = struct( ...
    'dt', 0.4, ...
    'totalTime', 1.0, ...
    'enableVisualization', false, ...
    'visualizerVisible', false, ...
    'waitTimeout', 2.0, ...
    'pathLibraryData', repmat(struct(), 0, 1), ...
    'dynamicObstacleSchedule', repmat(struct(), 0, 1));

simulation = sim.Simulation(simMap, [agvHigh; agvLow], [highTask; lowTask], config, schedulerObj, [], manager);
simulation.step();

assert(strcmp(agvLow.state, 'waiting') || strcmp(agvLow.state, 'moving'), ...
    'Lower-priority AGV should be delayed or otherwise adjusted by conflict resolution.');
assert(any(strcmp({simulation.eventLog.type}, 'conflict_resolved')), ...
    'Simulation step should log scheduler conflict handling.');
end
