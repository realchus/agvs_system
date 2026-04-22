function test_pathlibrary()
%TEST_PATHLIBRARY Validate candidate path library generation.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
dataDir = fullfile(projectRoot, 'data');
addpath(dataDir);

customMap = map.MapClass(zeros(5, 5), map.MapClass.defaultColors());
simpleTask = task.TaskClass(1, [1, 1], struct('name', 'Goal', 'position', [1, 5]), 1, 0, 'pending');

libraryEntry = pathplan.PathLibrary.generateLibrary(customMap, simpleTask, 2, 1.0, 1);
assert(libraryEntry.numGenerated == 2, 'Simple open map should generate two unique candidate paths.');
assert(isequal(libraryEntry.paths(1).nodes(1, :), [1, 1]), 'Candidate path should start at task start.');
assert(isequal(libraryEntry.paths(1).nodes(end, :), [1, 5]), 'Candidate path should end at the waypoint.');
assert(~isequal(libraryEntry.paths(1).nodes, libraryEntry.paths(2).nodes), ...
    'Candidate paths should differ after diversification.');
pathEdgeWindows = libraryEntry.paths(1).timeWindows( ...
    strcmp({libraryEntry.paths(1).timeWindows.windowType}, 'edge'));
pathNodeWindows = libraryEntry.paths(1).timeWindows( ...
    strcmp({libraryEntry.paths(1).timeWindows.windowType}, 'node'));
assert(numel(pathEdgeWindows) == size(libraryEntry.paths(1).nodes, 1) - 1, ...
    'Each edge in the path should have a corresponding edge time window.');
assert(numel(pathNodeWindows) >= size(libraryEntry.paths(1).nodes, 1), ...
    'Candidate paths should include node occupancy time windows.');

taskFile = fullfile(dataDir, 'task_list.mat');
if ~isfile(taskFile)
    createTaskList();
end

tasks = task.TaskParser(taskFile);
warehouseMap = map.MapClass.createDefaultMap();
batchLibrary = pathplan.PathLibrary.generateForTasks(warehouseMap, tasks(1:2), 2, 1.0);
assert(numel(batchLibrary) == 2, 'Batch generator should return one library entry per task.');
assert(batchLibrary(1).numGenerated >= 1 && batchLibrary(1).numGenerated <= 2, ...
    'Generated path count should stay within the requested bounds.');
assert(isequal(batchLibrary(2).paths(1).nodes(1, :), tasks(2).start), ...
    'Batch library should preserve task start positions.');

createPathLibrary();
loadedData = load(fullfile(dataDir, 'path_library.mat'));
assert(isfield(loadedData, 'pathLibraryData'), 'Saved path library MAT file is missing pathLibraryData.');
assert(numel(loadedData.pathLibraryData) == numel(tasks), 'Saved path library should include every task.');

disp('test_pathlibrary passed');
end
