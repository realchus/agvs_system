function test_agv()
%TEST_AGV Validate AGV movement and load/unload timing logic.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

vehicle = agv.AGVClass(1, [1, 1], 1.0);
vehicle.assignPath([1, 1; 1, 2; 1, 3]);

[reachedNode, position] = vehicle.move(0.5);
assert(~reachedNode, 'AGV should not reach the next node in half a second.');
assert(norm(position - [1, 1.5]) < 1e-9, 'AGV position after partial move is incorrect.');
assert(strcmp(vehicle.state, 'moving'), 'AGV should remain in moving state while path is active.');

[reachedNode, position] = vehicle.move(0.5);
assert(reachedNode, 'AGV should reach the first target node after one second.');
assert(norm(position - [1, 2]) < 1e-9, 'AGV failed to arrive at the expected node.');

[reachedNode, position] = vehicle.move(1.0);
assert(reachedNode, 'AGV should reach the final node.');
assert(norm(position - [1, 3]) < 1e-9, 'AGV final position is incorrect.');
assert(strcmp(vehicle.state, 'idle'), 'AGV should return to idle after finishing the path.');
assert(isempty(vehicle.path), 'Path should be cleared after traversal completes.');

completed = vehicle.load(2.0);
assert(~completed, 'Loading should not complete before full duration elapses.');
assert(strcmp(vehicle.state, 'loading'), 'AGV should be in loading state.');
assert(abs(vehicle.operationRemainingTime - 3.0) < 1e-9, 'Loading remaining time is incorrect.');

completed = vehicle.load(3.0);
assert(completed, 'Loading should complete after five seconds in total.');
assert(strcmp(vehicle.state, 'loaded'), 'AGV should enter loaded state after loading.');
assert(vehicle.isLoaded, 'AGV cargo flag should be true after loading.');

completed = vehicle.unload(4.0);
assert(~completed, 'Unloading should still be in progress after four seconds.');
assert(strcmp(vehicle.state, 'unloading'), 'AGV should be in unloading state.');
assert(abs(vehicle.operationRemainingTime - 1.0) < 1e-9, 'Unloading remaining time is incorrect.');

completed = vehicle.unload(1.0);
assert(completed, 'Unloading should complete after five seconds in total.');
assert(strcmp(vehicle.state, 'idle'), 'AGV should return to idle after unloading.');
assert(~vehicle.isLoaded, 'AGV cargo flag should be false after unloading.');

vehicle.updateState('fault');
assert(strcmp(vehicle.state, 'fault'), 'State update did not apply.');
assert(vehicle.operationRemainingTime == 0.0, 'Non-operation states should clear operation timers.');

agvStruct = vehicle.toStruct();
assert(agvStruct.id == 1 && isequal(agvStruct.position, [1, 3]), ...
    'Serialized AGV struct is missing expected values.');

defaultPool = agv.AGVClass.createDefaultPool();
assert(numel(defaultPool) == 3, 'Default AGV pool should contain three vehicles.');
assert(isequal(defaultPool(2).position, [1, 32]), 'Default AGV pool positions are incorrect.');


jsonPath = fullfile(tempdir, 'test_agv_pool.json');
jsonPayload = struct( ...
    'id', {11, 12}, ...
    'position', {[2, 3], [4, 5]}, ...
    'speed', {1.2, 0.8}, ...
    'state', {'idle', 'waiting'});
fid = fopen(jsonPath, 'w');
assert(fid > 0, 'Failed to open temporary AGV JSON file.');
fprintf(fid, '%s', jsonencode(jsonPayload));
fclose(fid);

parsedPool = agv.AGVPoolParser(jsonPath);
assert(numel(parsedPool) == 2, 'AGV JSON parser should load two vehicles.');
assert(parsedPool(1).id == 11 && isequal(parsedPool(1).position, [2, 3]), ...
    'AGV JSON parser failed to load first AGV fields.');
assert(abs(parsedPool(2).speed - 0.8) < 1e-9 && strcmp(parsedPool(2).state, 'waiting'), ...
    'AGV JSON parser failed to load state/speed aliases.');

delete(jsonPath);

disp('test_agv passed');
end
