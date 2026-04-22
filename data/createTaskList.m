function createTaskList()
%CREATETASKLIST Generate and save the default task definition files.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

outputDir = fullfile(projectRoot, 'data');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

leftPickupWindow = windowCells(1:20);
rightLoadingWindow = windowCells(43:62);

taskListData = [ ...
    buildTask(1, [5, 5], ...
        {'左侧拣货窗口', leftPickupWindow(10, :)}, 1, 0); ...
    buildTask(2, [9, 15], ...
        {'左侧拣货窗口', leftPickupWindow(15, :)}, 1, 10); ...
    buildTask(3, rightLoadingWindow(13, :), ...
        {'货架3', [13, 25]}, 1, 20); ...
    buildTask(4, rightLoadingWindow(18, :), ...
        {'货架4', [17, 35]}, 1, 90); ...
    buildTask(5, [21, 45], ...
        {'左侧拣货窗口', leftPickupWindow(5, :)}, 1, 40) ...
    ];

jsonText = jsonencode(taskListData, PrettyPrint=true);
fid = fopen(fullfile(outputDir, 'task_list.json'), 'w');
if fid == -1
    error('createTaskList:OpenFailed', 'Failed to open task_list.json for writing.');
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', jsonText);

save(fullfile(outputDir, 'task_list.mat'), 'taskListData');
disp('Saved data/task_list.json');
disp('Saved data/task_list.mat');
end

function taskData = buildTask(id, startPos, waypointCells, priority, requestTime)
waypoints = repmat(struct('name', '', 'position', [0, 0]), size(waypointCells, 1), 1);
for i = 1:size(waypointCells, 1)
    waypoints(i, 1) = struct( ...
        'name', waypointCells{i, 1}, ...
        'position', waypointCells{i, 2});
end

taskData = struct( ...
    'id', id, ...
    'start', startPos, ...
    'waypoints', waypoints, ...
    'priority', priority, ...
    'requestTime', requestTime, ...
    'status', 'pending');
end

function cells = windowCells(cols)
%WINDOWCELLS Return concrete window cells on the second map row.
%   The left picking window uses row 2, columns 1..20; the right loading
%   window uses row 2, columns 43..62. Each task selects one grid cell from
%   the corresponding window area as its executable start or end point.
cells = [2 * ones(numel(cols), 1), cols(:)];
end
