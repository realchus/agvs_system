function test_timewindow()
%TEST_TIMEWINDOW Validate time window reservation and conflict detection.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

manager = timewindow.TimeWindowManager();

windowA = manager.addTimeWindow([3, 3, 3, 4], 0.0, 1.0, 1, 0);
[hasConflict, info] = manager.detectConflict(struct( ...
    'edgeIndex', [3, 3, 3, 4], ...
    'startTime', 0.5, ...
    'endTime', 1.5, ...
    'agvId', 2, ...
    'direction', 0));
assert(hasConflict, 'Overlapping use of the same edge should conflict.');
assert(strcmp(info.type, 'same_direction_overlap'), 'Conflict type should mark same-direction overlap.');
assert(windowA.agvId == 1, 'Added window should preserve AGV id.');

[hasConflict, info] = manager.detectConflict(struct( ...
    'edgeIndex', [3, 4, 3, 3], ...
    'startTime', 0.25, ...
    'endTime', 0.75, ...
    'agvId', 3, ...
    'direction', 2));
assert(hasConflict, 'Opposite-direction use of the same edge should conflict.');
assert(strcmp(info.type, 'opposite_direction_overlap'), ...
    'Conflict type should mark opposite-direction overlap.');

[reservedWindows, conflictInfo] = manager.reservePath(4, [1, 1; 1, 2; 1, 3], 2.0, 1.0);
assert(isempty(conflictInfo), 'Conflict info should be empty when reservation succeeds.');
edgeWindows = reservedWindows(strcmp({reservedWindows.windowType}, 'edge'));
nodeWindows = reservedWindows(strcmp({reservedWindows.windowType}, 'node'));
assert(numel(edgeWindows) == 2, 'Path reservation should create one edge window per edge.');
assert(numel(nodeWindows) == 4, 'Path reservation should create node occupancy windows for segment endpoints.');
assert(abs(edgeWindows(1).startTime - 2.0) < 1e-9 && abs(edgeWindows(1).endTime - 3.0) < 1e-9, ...
    'First reserved edge timing is incorrect.');
assert(abs(edgeWindows(2).startTime - 3.0) < 1e-9 && abs(edgeWindows(2).endTime - 4.0) < 1e-9, ...
    'Second reserved edge timing is incorrect.');

[reservedNodeBlocked, nodeConflictInfo] = manager.reservePath(7, [2, 2; 1, 2; 1, 1], 2.0, 1.0);
assert(isempty(reservedNodeBlocked), 'Path with overlapping node occupancy should not be reserved.');
assert(~isempty(nodeConflictInfo), 'Node conflict details should be returned when reservation fails.');
assert(strcmp(nodeConflictInfo.type, 'node_overlap'), ...
    'Simultaneous use of the same node should be reported as node_overlap.');

[reservedWindowsBlocked, conflictInfo] = manager.reservePath(5, [1, 3; 1, 2], 3.2, 1.0);
assert(isempty(reservedWindowsBlocked), 'Conflicting path should not be reserved.');
assert(~isempty(conflictInfo), 'Conflict details should be returned when reservation fails.');
assert(any(strcmp(conflictInfo.type, {'opposite_direction_overlap', 'node_overlap'})), ...
    'Reverse use of the reserved edge or its endpoint should be reported as a conflict.');

[reservedWithDwell, dwellConflict] = manager.reservePath(8, [4, 1; 4, 2], 5.0, 1.0, [2.0; 0.0]);
assert(isempty(dwellConflict), 'Reservation with node dwell should succeed on an independent path.');
nodeMask = strcmp({reservedWithDwell.windowType}.', 'node');
startNodeMask = arrayfun(@(w) isequal(w.nodeIndex, [4, 1]), reservedWithDwell);
dwellNodeWindows = reservedWithDwell(nodeMask & startNodeMask);
assert(any(abs([dwellNodeWindows.startTime] - 5.0) < 1e-9 & ...
    abs([dwellNodeWindows.endTime] - 7.0) < 1e-9), ...
    'Node dwell should reserve the service node for the requested duration.');

[hasConflict, ~] = manager.detectConflict(struct( ...
    'edgeIndex', [5, 5, 5, 6], ...
    'startTime', 10.0, ...
    'endTime', 11.0, ...
    'agvId', 6, ...
    'direction', 0));
assert(~hasConflict, 'Independent edge usage should not conflict.');

manager.releasePath(4, [1, 1; 1, 2; 1, 3]);
remainingAgvIds = [manager.timeWindows.agvId];
assert(~any(remainingAgvIds == 4), 'Released path windows should be removed from the manager.');
assert(any(remainingAgvIds == 1), 'Unrelated reservations should remain after releasing a path.');

manager.releasePath(1);
manager.releasePath(8);
assert(isempty(manager.timeWindows), 'Releasing an AGV without a path should remove all of its windows.');

disp('test_timewindow passed');
end
