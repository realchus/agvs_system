function test_map()
%TEST_MAP Validate the warehouse map model and occupancy logic.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

warehouseMap = map.MapClass.createDefaultMap();

assert(isequal(size(warehouseMap.baseGrid), [28, 62]), 'Default base grid size mismatch.');
assert(warehouseMap.grid(1, 25) == 2, 'AGV 1 spawn was not applied.');
assert(warehouseMap.baseGrid(1, 25) == 0, 'Base grid should remain free at AGV spawns.');
assert(~warehouseMap.isPassable(1, 1, 1, []), 'Blocked platform cell should not be passable.');
assert(warehouseMap.isPassable(3, 11, 1, []), 'Free lane should be passable.');
assert(warehouseMap.isPassable(1, 25, 1, []), 'AGV should be allowed to stay on its own cell.');
assert(~warehouseMap.isPassable(1, 25, 2, []), 'Other AGVs should not pass through occupied cells.');

warehouseMap.registerTaskTarget(101, [5, 3]);
assert(warehouseMap.isPassable(5, 3, 1, 101), 'Registered task target shelf should be temporarily passable.');
assert(~warehouseMap.isPassable(5, 3, 1, 102), 'Unregistered task should not unlock shelf cells.');

warehouseMap.setAGVOccupancy(1, 3, 25);
assert(warehouseMap.grid(3, 25) == 2, 'Moved AGV cell should become occupied.');
assert(warehouseMap.grid(1, 25) == 0, 'Previous AGV cell should be restored after movement.');

warehouseMap.clearAGVOccupancy(1);
assert(warehouseMap.grid(3, 25) == 0, 'Cleared AGV occupancy should restore the base cell value.');

white = warehouseMap.getColor(3, 11);
assert(isequal(white, [1, 1, 1]), 'Free lane color should be white.');

mapConfig = warehouseMap.toStruct();
assert(isfield(mapConfig, 'baseGrid') && isfield(mapConfig, 'occupancy'), ...
    'Serialized map config is missing required fields.');

disp('test_map passed');
end
