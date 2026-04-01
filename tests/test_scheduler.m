function test_scheduler()
%TEST_SCHEDULER Validate task ordering, assignment, merge, and emergency insert.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
dataDir = fullfile(projectRoot, 'data');
addpath(dataDir);

if ~isfile(fullfile(dataDir, 'path_library.mat'))
    createPathLibrary();
end

warehouseMap = map.MapClass.createDefaultMap();
agvPool = agv.AGVClass.createDefaultPool();
tasks = task.TaskParser(fullfile(dataDir, 'task_list.mat'));
schedulerObj = scheduler.SchedulerClass(warehouseMap, agvPool, tasks);

orderedTasks = schedulerObj.updatePriority(50.0);
assert(orderedTasks(1).id == 1, 'Earliest request time should rank first when priorities tie.');

nextTask = schedulerObj.getNextTask(50.0);
assert(nextTask.id == 1, 'getNextTask should return the top-ranked task.');

[success, assignedPath, assignedWindows, sourceLabel] = schedulerObj.assignToAGV(agvPool(1), tasks(1), 0.0);
assert(success, 'assignToAGV should succeed for the first task.');
assert(strcmp(sourceLabel, 'library'), 'Scheduler should consult the path library before fallback planning.');
assert(~isempty(assignedPath), 'Assigned path should not be empty.');
assert(numel(assignedWindows) == size(assignedPath, 1) - 1, ...
    'Reserved windows should match the number of path edges.');
assert(strcmp(tasks(1).status, 'assigned'), 'Assigned task status should be updated.');
assert(isa(agvPool(1).currentTask, 'task.TaskClass') && agvPool(1).currentTask(1).id == 1, ...
    'AGV should reference the assigned task.');
assert(~isempty(schedulerObj.timeWindowsGlobal), 'Global time windows should be updated after assignment.');

simpleMap = map.MapClass(zeros(3, 6), map.MapClass.defaultColors());
agvSingle = agv.AGVClass(1, [1, 1], 1.0);
primaryTask = task.TaskClass(101, [1, 1], struct('name', 'GoalA', 'position', [1, 6]), 1, 0, 'assigned');
mergeTask = task.TaskClass(102, [1, 3], struct('name', 'GoalB', 'position', [1, 5]), 1, 5, 'pending');
offPathTask = task.TaskClass(103, [2, 2], struct('name', 'GoalC', 'position', [2, 4]), 1, 10, 'pending');
agvSingle.assignTask(primaryTask);
agvSingle.assignPath([1, 1; 1, 2; 1, 3; 1, 4; 1, 5; 1, 6]);
simpleScheduler = scheduler.SchedulerClass(simpleMap, agvSingle, [primaryTask; mergeTask; offPathTask], repmat(struct(), 0, 1));

mergedTasks = simpleScheduler.mergePickup(agvSingle);
assert(numel(mergedTasks) == 1 && mergedTasks(1).id == 102, 'Only along-path tasks should be merged.');
assert(strcmp(mergeTask.status, 'assigned'), 'Merged task status should update to assigned.');
assert(numel(agvSingle.currentTask) == 2, 'Merged task should be appended to AGV currentTask.');

urgentTask = task.TaskClass(104, [1, 2], struct('name', 'GoalD', 'position', [1, 4]), 1, 100, 'pending');
simpleScheduler.taskList = [simpleScheduler.taskList; urgentTask];
simpleScheduler.emergencyInsert(urgentTask);
nextUrgent = simpleScheduler.getNextTask(0.0);
assert(nextUrgent.id == 104, 'Emergency task should move to the front of the queue.');

disp('test_scheduler passed');
end
