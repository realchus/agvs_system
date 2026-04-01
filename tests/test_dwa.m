function test_dwa()
%TEST_DWA Validate local avoidance and conflict-triggered replanning.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

testLocalAvoidanceUpdatesPathAndTimeWindows();
testConflictAfterAvoidanceTriggersReplan();

disp('test_dwa passed');
end

function testLocalAvoidanceUpdatesPathAndTimeWindows()
testMap = map.MapClass(zeros(5, 5), map.MapClass.defaultColors());
manager = timewindow.TimeWindowManager();
vehicle = agv.AGVClass(1, [3, 1], 1.0);
taskObj = task.TaskClass(401, [3, 1], struct('name', 'Goal', 'position', [3, 5]), 1, 0, 'assigned');

originalPath = [3, 1; 3, 2; 3, 3; 3, 4; 3, 5];
vehicle.assignTask(taskObj);
vehicle.assignPath(originalPath);
[reservedWindows, conflictInfo] = manager.reservePath(vehicle.id, originalPath, 0.0, vehicle.speed);
assert(isempty(conflictInfo), 'Initial reservation should succeed.');
vehicle.setTimeWindows(reservedWindows);

context = struct( ...
    'map', testMap, ...
    'dynamicObstacles', [3, 2], ...
    'timeWindowManager', manager, ...
    'currentTime', 0.0);

[reachedNode, position, moveInfo] = vehicle.move(1.0, context);
assert(reachedNode, 'Vehicle should still advance after local avoidance.');
assert(moveInfo.avoidanceTriggered, 'Dynamic obstacle should trigger local avoidance.');
assert(strcmp(moveInfo.strategy, 'dwa'), 'Local detour without reservation conflict should keep DWA strategy.');
assert(~agv.AGVClass.containsRow(vehicle.path, [3, 2]), 'Avoidance path should not traverse the blocked node.');
assert(~isempty(manager.invalidatedWindows), 'Original reserved windows should be marked invalid after obstacle detection.');
assert(~isempty(vehicle.timeWindows), 'Vehicle should receive updated reservations for the detour path.');
assert(any(all(round(position) == [2, 1], 2)) || any(all(round(position) == [4, 1], 2)), ...
    'Vehicle should move onto a detour branch instead of the blocked cell.');
end

function testConflictAfterAvoidanceTriggersReplan()
testMap = map.MapClass(zeros(5, 5), map.MapClass.defaultColors());
manager = timewindow.TimeWindowManager();

vehicle = agv.AGVClass(1, [3, 1], 1.0);
taskObj = task.TaskClass(402, [3, 1], struct('name', 'Goal', 'position', [3, 5]), 1, 0, 'assigned');
vehicle.assignTask(taskObj);

originalPath = [3, 1; 3, 2; 3, 3; 3, 4; 3, 5];
vehicle.assignPath(originalPath);
[reservedWindows, conflictInfo] = manager.reservePath(vehicle.id, originalPath, 0.0, vehicle.speed);
assert(isempty(conflictInfo), 'Initial reservation should succeed.');
vehicle.setTimeWindows(reservedWindows);

otherAgv = agv.AGVClass(2, [2, 1], 1.0);
otherPath = [2, 1; 2, 2; 2, 3; 2, 4; 2, 5];
[otherWindows, otherConflict] = manager.reservePath(otherAgv.id, otherPath, 1.0, otherAgv.speed);
assert(isempty(otherConflict), 'Other AGV reservation should succeed.');
otherAgv.assignPath(otherPath);
otherAgv.setTimeWindows(otherWindows);

context = struct( ...
    'map', testMap, ...
    'dynamicObstacles', [3, 2], ...
    'timeWindowManager', manager, ...
    'currentTime', 0.0);

[~, position, moveInfo] = vehicle.move(1.0, context);
assert(moveInfo.avoidanceTriggered, 'Dynamic obstacle should still trigger avoidance.');
assert(moveInfo.conflictDetected, 'The first detour attempt should detect a time-window conflict.');
assert(moveInfo.replannedAfterConflict, 'Conflict after local avoidance should trigger a remaining-path replan.');
assert(strcmp(moveInfo.strategy, 'replan'), 'Conflict-handling branch should mark the move as replanned.');
assert(agv.AGVClass.containsRow(vehicle.path, [4, 1]), ...
    'Replanned path should route around both the obstacle and the conflicting reserved edge.');
assert(~agv.AGVClass.containsRow(vehicle.path, [2, 2]), ...
    'Replanned path should avoid the conflicting upper corridor.');
assert(~isempty(vehicle.timeWindows), 'Vehicle should end with a valid reservation after replanning.');
assert(any(all(round(position) == [4, 1], 2)), 'Vehicle should move onto the replanned lower corridor.');
end
