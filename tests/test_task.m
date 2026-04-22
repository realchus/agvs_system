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
assert(isequal(firstTask.start, [5, 5]), 'First task start position is incorrect.');
assert(firstTask.priority == 1, 'Priority should be initialized to 1.');
assert(firstTask.requestTime == 0, 'Request time initialization is incorrect.');
assert(strcmp(firstTask.status, 'pending'), 'Initial task status should be pending.');

waypointNames = firstTask.getWaypointNames();
assert(strcmp(waypointNames{1}, '左侧拣货窗口'), 'Waypoint name parsing failed.');

positions = firstTask.getWaypointPositions();
assert(isequal(positions(1, :), [2, 10]), 'Waypoint positions were not parsed correctly.');

thirdTask = matTasks(3);
assert(thirdTask.requestTime == 20, 'Request time sequence should be preserved from MAT data.');
assert(isequal(thirdTask.start, [2, 55]), 'Third task should start from the right loading window.');
assert(strcmp(thirdTask.waypoints(1).name, '货架3'), ...
    'MAT parser did not preserve waypoint names.');

assert(matTasks(4).requestTime == 90, ...
    'Fourth task should be delayed to avoid opposite-direction default traffic.');

allStarts = reshape([matTasks.start], 2, []).';
allEnds = zeros(numel(matTasks), 2);
for i = 1:numel(matTasks)
    positions = matTasks(i).getWaypointPositions();
    allEnds(i, :) = positions(end, :);
end

assert(all(allStarts(:, 1) ~= 1), 'Task starts should no longer use AGV parking cells.');
assert(all(allEnds(:, 1) ~= 1), 'Task ends should no longer use AGV parking cells.');
assert(all(ismember(allStarts([3, 4], 2), 43:62) & allStarts([3, 4], 1) == 2), ...
    'Right loading window starts must be on row 2, columns 43..62.');
assert(all(ismember(allEnds([1, 2, 5], 2), 1:20) & allEnds([1, 2, 5], 1) == 2), ...
    'Left picking window ends must be on row 2, columns 1..20.');

thirdTask.updateStatus('assigned');
assert(strcmp(thirdTask.status, 'assigned'), 'Task status update failed.');

taskStructs = task.TaskClass.toStructArray(matTasks);
assert(numel(taskStructs) == 5 && strcmp(taskStructs(5).status, 'pending'), ...
    'Task struct serialization failed.');

inputTasks = task.TaskParser(fullfile(projectRoot, 'input', 'task_list.json'));
assert(numel(inputTasks) == 30, 'Input task list should contain thirty tasks.');
inputStarts = reshape([inputTasks.start], 2, []).';
assert(all(inputStarts(:, 1) ~= 1), 'Input task starts should not use AGV parking cells.');
inputRequestTimes = [inputTasks.requestTime];
expectedPairedSchedule = 0:120:1680;
assert(isequal(inputRequestTimes(1:15), expectedPairedSchedule), ...
    'Outbound input tasks should use the paired 120-second release schedule.');
assert(isequal(inputRequestTimes(16:30), expectedPairedSchedule), ...
    'Inbound input tasks should be paired with outbound releases.');

disp('test_task passed');
end
