function test_task()
%TEST_TASK Validate task model parsing and persistence logic.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

dataDir = fullfile(projectRoot, 'data');
addpath(dataDir);
jsonPath = fullfile(dataDir, 'task_list.json');
matPath = fullfile(dataDir, 'task_list.mat');

if ~isfile(jsonPath) || ~isfile(matPath)
    createTaskList();
end

jsonTasks = task.TaskParser(jsonPath);
matTasks = task.TaskParser(matPath);

assert(numel(jsonTasks) == 5, 'JSON parser should load five tasks.');
assert(numel(matTasks) == 5, 'MAT parser should load five tasks.');

firstTask = jsonTasks(1);
assert(firstTask.id == 1, 'First task id is incorrect.');
assert(isequal(firstTask.start, [1, 25]), 'First task start position is incorrect.');
assert(firstTask.priority == 1, 'Priority should be initialized to 1.');
assert(firstTask.requestTime == 0, 'Request time initialization is incorrect.');
assert(strcmp(firstTask.status, 'pending'), 'Initial task status should be pending.');

waypointNames = firstTask.getWaypointNames();
assert(strcmp(waypointNames{1}, '货架1'), 'Waypoint name parsing failed.');

positions = firstTask.getWaypointPositions();
assert(isequal(positions(2, :), [2, 10]), 'Waypoint positions were not parsed correctly.');

thirdTask = matTasks(3);
assert(thirdTask.requestTime == 20, 'Request time sequence should be preserved from MAT data.');
assert(strcmp(thirdTask.waypoints(1).name, '右侧上货窗口'), ...
    'MAT parser did not preserve waypoint names.');

thirdTask.updateStatus('assigned');
assert(strcmp(thirdTask.status, 'assigned'), 'Task status update failed.');

taskStructs = task.TaskClass.toStructArray(matTasks);
assert(numel(taskStructs) == 5 && strcmp(taskStructs(5).status, 'pending'), ...
    'Task struct serialization failed.');

disp('test_task passed');
end
