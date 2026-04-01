function test_astar()
%TEST_ASTAR Validate A* path planning on the warehouse map.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

warehouseMap = map.MapClass.createDefaultMap();

path1 = pathplan.AStar(warehouseMap, [3, 11], [3, 20], 1, []);
assert(~isempty(path1), 'Planner should find a straight corridor path.');
assert(isequal(path1(1, :), [3, 11]), 'Path must start at the requested start node.');
assert(isequal(path1(end, :), [3, 20]), 'Path must end at the requested goal node.');
assert(all(sum(abs(diff(path1, 1, 1)), 2) == 1), 'Path must use 4-neighbor moves only.');

path2 = pathplan.AStar(warehouseMap, [3, 3], [7, 3], 1, []);
assert(~isempty(path2), 'Planner should detour around shelf obstacles.');
assert(~any(all(path2 == [5, 3], 2)), 'Path must not pass through blocked shelf cells.');
assert(all(arrayfun(@(idx) warehouseMap.isPassable(path2(idx, 1), path2(idx, 2), 1, []), 1:size(path2, 1))), ...
    'Detour path contains a non-passable node.');

warehouseMap.setAGVOccupancy(99, 3, 15);
path3 = pathplan.AStar(warehouseMap, [3, 11], [3, 19], 2, []);
assert(~isempty(path3), 'Planner should find a detour around AGV occupancy.');
assert(~any(all(path3 == [3, 15], 2)), 'Path must avoid occupied cells from other AGVs.');

warehouseMap.registerTaskTarget(301, [5, 5]);
path4 = pathplan.AStar(warehouseMap, [3, 5], [5, 5], 1, 301);
assert(~isempty(path4), 'Planner should allow a registered target shelf as a goal.');
assert(isequal(path4(end, :), [5, 5]), 'Target shelf path must terminate on the unlocked goal.');

path5 = pathplan.AStar(warehouseMap, [3, 5], [5, 6], 1, []);
assert(isempty(path5), 'Planner should not enter blocked shelf cells without task authorization.');

disp('test_astar passed');
end
