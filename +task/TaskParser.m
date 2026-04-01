function tasks = TaskParser(sourcePath)
%TASKPARSER Load task definitions from a MAT or JSON file.
% Input:
%   sourcePath - Path to .mat or .json task definition file.
% Output:
%   tasks      - Column vector of task.TaskClass objects.

if nargin < 1 || isempty(sourcePath)
    sourcePath = fullfile(pwd, 'data', 'task_list.mat');
end

validateattributes(sourcePath, {'char', 'string'}, {'nonempty'});
sourcePath = char(string(sourcePath));

if ~isfile(sourcePath)
    error('TaskParser:FileNotFound', 'Task data file not found: %s', sourcePath);
end

[~, ~, ext] = fileparts(sourcePath);
switch lower(ext)
    case '.mat'
        rawTasks = loadFromMat(sourcePath);
    case '.json'
        rawTasks = loadFromJson(sourcePath);
    otherwise
        error('TaskParser:UnsupportedFormat', ...
            'Unsupported task file format: %s. Use .mat or .json.', ext);
end

tasks = buildTaskArray(rawTasks);
end

function rawTasks = loadFromMat(sourcePath)
data = load(sourcePath);
candidateFields = {'taskListData', 'taskList', 'tasks'};
for i = 1:numel(candidateFields)
    if isfield(data, candidateFields{i})
        rawTasks = data.(candidateFields{i});
        return;
    end
end

error('TaskParser:MissingTaskVariable', ...
    'MAT file %s does not contain taskListData, taskList, or tasks.', sourcePath);
end

function rawTasks = loadFromJson(sourcePath)
rawText = fileread(sourcePath);
rawTasks = jsondecode(rawText);
end

function tasks = buildTaskArray(rawTasks)
if isempty(rawTasks)
    tasks = repmat(task.TaskClass(1, [1, 1], [], 1, 0, 'pending'), 0, 1);
    return;
end

if ~isstruct(rawTasks)
    error('TaskParser:InvalidTaskData', 'Parsed task data must be a struct array.');
end

tasks = repmat(task.TaskClass(1, [1, 1], [], 1, 0, 'pending'), numel(rawTasks), 1);
for i = 1:numel(rawTasks)
    taskId = readField(rawTasks(i), {'id'});
    startPos = readField(rawTasks(i), {'start'});
    waypoints = readField(rawTasks(i), {'waypoints'});
    priority = readField(rawTasks(i), {'priority'}, 1);
    requestTime = readField(rawTasks(i), {'requestTime', 'request_time'}, 0);
    status = readField(rawTasks(i), {'status'}, 'pending');

    tasks(i, 1) = task.TaskClass(taskId, startPos, waypoints, priority, requestTime, status);
end
end

function value = readField(taskData, names, defaultValue)
if nargin < 3
    defaultValue = [];
end

for i = 1:numel(names)
    if isfield(taskData, names{i}) && ~isempty(taskData.(names{i}))
        value = taskData.(names{i});
        return;
    end
end

if ~isempty(defaultValue)
    value = defaultValue;
    return;
end

error('TaskParser:MissingField', 'Task data is missing required field: %s', names{1});
end
