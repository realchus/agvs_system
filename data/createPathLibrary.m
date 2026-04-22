function createPathLibrary()
%CREATEPATHLIBRARY Generate and save the default path library file.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

dataDir = fullfile(projectRoot, 'data');
addpath(dataDir);
if ~exist(dataDir, 'dir')
    mkdir(dataDir);
end

inputTaskPath = fullfile(projectRoot, 'input', 'task_list.json');
taskPath = fullfile(dataDir, 'task_list.mat');
if isfile(inputTaskPath)
    taskPath = inputTaskPath;
elseif ~isfile(taskPath)
    createTaskList();
end

tasks = task.TaskParser(taskPath);
warehouseMap = map.MapClass.createDefaultMap();
pathLibraryData = pathplan.PathLibrary.generateForTasks(warehouseMap, tasks, 3, 1.0); %#ok<NASGU>

save(fullfile(dataDir, 'path_library.mat'), 'pathLibraryData');
disp('Saved data/path_library.mat');
end
