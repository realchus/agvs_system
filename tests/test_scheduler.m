function test_scheduler()
%TEST_SCHEDULER Validate scheduling, assignment, merge, and conflict handling.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
dataDir = fullfile(projectRoot, 'data');
addpath(dataDir);

if ~isfile(fullfile(dataDir, 'path_library.mat'))
    createPathLibrary();
end

warehouseMap = map.MapClass.createDefaultMap();
agvPool = agv.AGVClass.createDefaultPool();
tasks = task.TaskParser(fullfile(dataDir, 'task_list.mat'));
schedulerObj = scheduler.SchedulerClass(warehouseMap, agvPool, tasks);

orderedTasks = schedulerObj.updatePriority(50.0);
assert(orderedTasks(1).id == 1, 'Earliest request time should rank first when priorities tie.');

nextTask = schedulerObj.getNextTask(50.0);
assert(nextTask.id == 1, 'getNextTask should return the top-ranked task.');

[success, assignedPath, assignedWindows, sourceLabel] = schedulerObj.assignToAGV(agvPool(1), tasks(1), 0.0);
assert(success, 'assignToAGV should succeed for the first task.');
assert(strcmp(sourceLabel, 'library'), 'Scheduler should consult the path library before fallback planning.');
assert(~isempty(assignedPath), 'Assigned path should not be empty.');
assignedEdgeWindows = assignedWindows(strcmp({assignedWindows.windowType}, 'edge'));
assignedNodeWindows = assignedWindows(strcmp({assignedWindows.windowType}, 'node'));
assert(numel(assignedEdgeWindows) == size(assignedPath, 1) - 1, ...
    'Reserved edge windows should match the number of path edges.');
assert(numel(assignedNodeWindows) >= size(assignedPath, 1), ...
    'Reserved windows should include node occupancy windows.');
assert(any(strcmp({assignedNodeWindows.windowType}, 'node') & ...
    arrayfun(@(w) isequal(w.nodeIndex, tasks(1).start) && ...
    abs((w.endTime - w.startTime) - agvPool(1).loadDuration) < 1e-9, assignedNodeWindows).'), ...
    'Task assignment should reserve the loading node for the configured load duration.');
assert(strcmp(tasks(1).status, 'assigned'), 'Assigned task status should be updated.');
assert(isa(agvPool(1).currentTask, 'task.TaskClass') && agvPool(1).currentTask(1).id == 1, ...
    'AGV should reference the assigned task.');
assert(~isempty(schedulerObj.timeWindowsGlobal), 'Global time windows should be updated after assignment.');

simpleMap = map.MapClass(zeros(3, 6), map.MapClass.defaultColors());
agvSingle = agv.AGVClass(1, [1, 1], 1.0);
primaryTask = task.TaskClass(101, [1, 1], struct('name', 'GoalA', 'position', [1, 6]), 1, 0, 'assigned');
mergeTask = task.TaskClass(102, [1, 3], struct('name', 'GoalB', 'position', [1, 5]), 1, 5, 'pending');
offPathTask = task.TaskClass(103, [2, 2], struct('name', 'GoalC', 'position', [2, 4]), 1, 10, 'pending');
agvSingle.assignTask(primaryTask);
agvSingle.assignPath([1, 1; 1, 2; 1, 3; 1, 4; 1, 5; 1, 6]);
simpleScheduler = scheduler.SchedulerClass(simpleMap, agvSingle, [primaryTask; mergeTask; offPathTask], repmat(struct(), 0, 1));

mergedTasks = simpleScheduler.mergePickup(agvSingle);
assert(numel(mergedTasks) == 1 && mergedTasks(1).id == 102, 'Only along-path tasks should be merged.');
assert(strcmp(mergeTask.status, 'assigned'), 'Merged task status should update to assigned.');
assert(numel(agvSingle.currentTask) == 2, 'Merged task should be appended to AGV currentTask.');

urgentTask = task.TaskClass(104, [1, 2], struct('name', 'GoalD', 'position', [1, 4]), 1, 100, 'pending');
simpleScheduler.taskList = [simpleScheduler.taskList; urgentTask];
simpleScheduler.emergencyInsert(urgentTask);
nextUrgent = simpleScheduler.getNextTask(0.0);
assert(nextUrgent.id == 104, 'Emergency task should move to the front of the queue.');

testConflictResolvedByWaiting();
testConflictResolvedByReplan();

disp('test_scheduler passed');
end

function testConflictResolvedByWaiting()
conflictMap = map.MapClass(zeros(3, 5), map.MapClass.defaultColors());
highTask = task.TaskClass(201, [2, 1], struct('name', 'HighGoal', 'position', [2, 4]), 5, 0, 'assigned');
lowTask = task.TaskClass(202, [2, 2], struct('name', 'LowGoal', 'position', [2, 5]), 1, 10, 'assigned');

agvHigh = agv.AGVClass(1, [2, 1], 1.0);
agvLow = agv.AGVClass(2, [2, 2], 1.0);

highPath = [2, 1; 2, 2; 2, 3; 2, 4];
lowPath = [2, 2; 2, 3; 2, 4; 2, 5];
highWindows = reserveStandaloneWindows(agvHigh.id, highPath, 0.0, agvHigh.speed);
lowWindows = reserveStandaloneWindows(agvLow.id, lowPath, 1.0, agvLow.speed);

agvHigh.assignTask(highTask);
agvHigh.assignPath(highPath);
agvHigh.setTimeWindows(highWindows);

agvLow.assignTask(lowTask);
agvLow.assignPath(lowPath);
agvLow.setTimeWindows(lowWindows);

manager = timewindow.TimeWindowManager();
manager.timeWindows = [highWindows; lowWindows];
manager.refreshIndexes();

waitScheduler = scheduler.SchedulerClass( ...
    conflictMap, [agvHigh; agvLow], [highTask; lowTask], repmat(struct(), 0, 1), manager);

[resolved, resolutionInfo] = waitScheduler.handleConflict(agvHigh, agvLow, 0.0, 2.0);
assert(resolved, 'Conflict should be solvable by delaying the lower-priority AGV.');
assert(strcmp(resolutionInfo.strategy, 'wait'), 'Conflict should choose the wait strategy.');
assert(resolutionInfo.keptAgvId == agvHigh.id && resolutionInfo.adjustedAgvId == agvLow.id, ...
    'Higher-priority AGV should keep the route.');
assert(abs(resolutionInfo.delay - 1.0) < 1e-9, 'Expected one-second delayed start for the lower-priority AGV.');
assert(strcmp(agvLow.state, 'waiting'), 'Lower-priority AGV should be marked as waiting after delay.');
assert(abs(agvLow.timeWindows(1).startTime - 2.0) < 1e-9, 'Delayed reservation should start after the conflicting edge clears.');
assertNoConflictBetweenAgvs(waitScheduler.timeWindowManager, agvHigh.timeWindows, agvLow.timeWindows);
end

function testConflictResolvedByReplan()
conflictMap = map.MapClass(zeros(5, 5), map.MapClass.defaultColors());
highTask = task.TaskClass(301, [3, 1], struct('name', 'HighGoal', 'position', [3, 4]), 10, 0, 'assigned');
lowTask = task.TaskClass(302, [3, 4], struct('name', 'LowGoal', 'position', [3, 1]), 1, 20, 'assigned');

agvHigh = agv.AGVClass(1, [3, 1], 1.0);
agvLow = agv.AGVClass(2, [3, 4], 1.0);

highPath = [3, 1; 3, 2; 3, 3; 3, 4];
lowDirectPath = [3, 4; 3, 3; 3, 2; 3, 1];
lowDetourPath = [3, 4; 2, 4; 2, 3; 2, 2; 2, 1; 3, 1];

highWindows = reserveStandaloneWindows(agvHigh.id, highPath, 0.0, agvHigh.speed);
lowWindows = reserveStandaloneWindows(agvLow.id, lowDirectPath, 0.0, agvLow.speed);

agvHigh.assignTask(highTask);
agvHigh.assignPath(highPath);
agvHigh.setTimeWindows(highWindows);

agvLow.assignTask(lowTask);
agvLow.assignPath(lowDirectPath);
agvLow.setTimeWindows(lowWindows);
agvLow.isLoaded = true;

manager = timewindow.TimeWindowManager();
manager.timeWindows = [highWindows; lowWindows];
manager.refreshIndexes();

pathLibraryData = buildLibraryEntry(lowTask, agvLow, lowDirectPath, lowDetourPath);
replanScheduler = scheduler.SchedulerClass( ...
    conflictMap, [agvHigh; agvLow], [highTask; lowTask], pathLibraryData, manager);

[resolved, resolutionInfo] = replanScheduler.handleConflict(agvHigh, agvLow, 0.0, 0.0);
assert(resolved, 'Conflict should be solvable by selecting an alternate route.');
assert(strcmp(resolutionInfo.strategy, 'replan'), 'Conflict should fall back to replanning when waiting is disallowed.');
assert(strcmp(resolutionInfo.sourceLabel, 'library'), 'Alternate route should come from the path library when available.');
assert(isequal(agvLow.path, lowDetourPath), 'Lower-priority AGV should switch to the library detour path.');
assertNoConflictBetweenAgvs(replanScheduler.timeWindowManager, agvHigh.timeWindows, agvLow.timeWindows);
end

function windows = reserveStandaloneWindows(agvId, pathNodes, startTime, speed)
manager = timewindow.TimeWindowManager();
[windows, conflictInfo] = manager.reservePath(agvId, pathNodes, startTime, speed);
assert(isempty(conflictInfo), 'Standalone path reservation should not conflict.');
end

function libraryEntry = buildLibraryEntry(taskObj, agvObj, primaryPath, alternatePath)
candidatePaths = repmat(struct( ...
    'pathId', 0, ...
    'nodes', zeros(0, 2), ...
    'timeWindows', timewindow.TimeWindowManager.emptyWindowArray(), ...
    'length', 0.0, ...
    'blockedNodesUsed', zeros(0, 2)), 2, 1);

candidatePaths(1) = buildCandidatePathStruct(1, primaryPath, agvObj.speed, agvObj.id);
candidatePaths(2) = buildCandidatePathStruct(2, alternatePath, agvObj.speed, agvObj.id);

libraryEntry = struct( ...
    'taskId', taskObj.id, ...
    'start', taskObj.start, ...
    'waypoints', taskObj.waypoints, ...
    'numRequested', 2, ...
    'numGenerated', 2, ...
    'speed', agvObj.speed, ...
    'paths', candidatePaths);
end

function candidate = buildCandidatePathStruct(pathId, nodes, speed, agvId)
windows = reserveStandaloneWindows(agvId, nodes, 0.0, speed);
deltas = diff(nodes, 1, 1);
candidate = struct( ...
    'pathId', double(pathId), ...
    'nodes', double(nodes), ...
    'timeWindows', windows, ...
    'length', double(sum(sqrt(sum(deltas .^ 2, 2)))), ...
    'blockedNodesUsed', zeros(0, 2));
end

function assertNoConflictBetweenAgvs(manager, windowsA, windowsB)
for i = 1:numel(windowsA)
    [hasConflict, ~] = manager.detectConflict(windowsA(i), windowsB);
    assert(~hasConflict, 'Resolved schedules should not contain edge-time conflicts.');
end
end
