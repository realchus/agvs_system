classdef SchedulerClass < handle
    %SCHEDULERCLASS Task scheduler for AGV assignment and queue management.
    %   The scheduler ranks pending tasks, assigns feasible routes to AGVs,
    %   and manages global time window reservations.

    properties
        map
        agvPool
        taskList
        taskQueue
        pathLibraryData
        timeWindowManager
        timeWindowsGlobal
        priorityWeight
        emergencyPriorityBoost
    end

    methods
        function obj = SchedulerClass(mapObj, agvPool, taskList, pathLibraryData, timeWindowManager)
            %SCHEDULERCLASS Construct a scheduler instance.
            % Inputs:
            %   mapObj            - map.MapClass instance.
            %   agvPool           - AGV object array.
            %   taskList          - Task object array.
            %   pathLibraryData   - Optional precomputed path library data.
            %   timeWindowManager - Optional global time window manager.
            if nargin < 1
                error('SchedulerClass:MissingMap', 'A map object is required.');
            end
            if nargin < 2 || isempty(agvPool)
                agvPool = agv.AGVClass.createDefaultPool();
            end
            if nargin < 3 || isempty(taskList)
                taskList = task.TaskParser(fullfile(pwd, 'data', 'task_list.mat'));
            end
            if nargin < 4 || isempty(pathLibraryData)
                pathLibraryData = scheduler.SchedulerClass.loadDefaultPathLibrary();
            end
            if nargin < 5 || isempty(timeWindowManager)
                timeWindowManager = timewindow.TimeWindowManager();
            end

            obj.map = mapObj;
            obj.agvPool = agvPool;
            obj.taskList = taskList;
            obj.pathLibraryData = pathLibraryData;
            obj.timeWindowManager = timeWindowManager;
            obj.timeWindowsGlobal = obj.timeWindowManager.timeWindows;
            obj.priorityWeight = 0.1;
            obj.emergencyPriorityBoost = 1000;
            obj.taskQueue = scheduler.SchedulerClass.emptyTaskArray();

            obj.updatePriority(0.0);
        end

        function orderedTasks = updatePriority(obj, currentTime)
            %UPDATEPRIORITY Re-rank pending tasks using weighted priority+FIFO.
            % Input:
            %   currentTime   - Scheduler time used for waiting-time weighting.
            % Output:
            %   orderedTasks  - Pending tasks sorted by effective priority.
            if nargin < 2 || isempty(currentTime)
                currentTime = 0.0;
            end

            pendingTasks = obj.getPendingTasks();
            if isempty(pendingTasks)
                obj.taskQueue = scheduler.SchedulerClass.emptyTaskArray();
                orderedTasks = obj.taskQueue;
                return;
            end

            basePriority = arrayfun(@(t) t.priority, pendingTasks);
            waitingTime = max(0.0, currentTime - arrayfun(@(t) t.requestTime, pendingTasks));
            effectivePriority = basePriority + obj.priorityWeight .* waitingTime;
            requestTimes = arrayfun(@(t) t.requestTime, pendingTasks);
            ids = arrayfun(@(t) t.id, pendingTasks);

            sortMatrix = [-effectivePriority(:), requestTimes(:), ids(:)];
            [~, order] = sortrows(sortMatrix, [1, 2, 3]);
            obj.taskQueue = pendingTasks(order);
            orderedTasks = obj.taskQueue;
        end

        function nextTask = getNextTask(obj, currentTime)
            %GETNEXTTASK Return the highest-ranked pending task.
            % Input:
            %   currentTime - Optional scheduler time.
            % Output:
            %   nextTask    - Top-ranked pending task, or [] if none exist.
            if nargin >= 2
                obj.updatePriority(currentTime);
            elseif isempty(obj.taskQueue)
                obj.updatePriority(0.0);
            end

            if isempty(obj.taskQueue)
                nextTask = [];
            else
                nextTask = obj.taskQueue(1);
            end
        end

        function [success, assignedPath, assignedWindows, sourceLabel] = assignToAGV(obj, agvObj, taskObj, startTime)
            %ASSIGNTOAGV Assign a task to an AGV and reserve time windows.
            % Inputs:
            %   agvObj    - AGV object to receive the task.
            %   taskObj   - Task object to assign.
            %   startTime - Optional path start time.
            % Outputs:
            %   success        - True when the assignment succeeds.
            %   assignedPath   - Path assigned to the AGV.
            %   assignedWindows - Reserved time window array.
            %   sourceLabel    - 'library' or 'astar'.
            if nargin < 4 || isempty(startTime)
                startTime = 0.0;
            end

            success = false;
            assignedPath = zeros(0, 2);
            assignedWindows = timewindow.TimeWindowManager.emptyWindowArray();
            sourceLabel = '';

            [candidatePaths, sourceLabel] = obj.getCandidatePaths(taskObj, agvObj);
            if isempty(candidatePaths)
                sourceLabel = 'astar';
                return;
            end

            for i = 1:numel(candidatePaths)
                [reservedWindows, conflictInfo] = obj.timeWindowManager.reservePath( ...
                    agvObj.id, candidatePaths(i).nodes, startTime, agvObj.speed);
                if ~isempty(conflictInfo)
                    continue;
                end

                taskObj.updateStatus('assigned');
                agvObj.assignTask(taskObj);
                agvObj.assignPath(candidatePaths(i).nodes);
                agvObj.setTimeWindows(reservedWindows);
                agvObj.updateState('moving');

                obj.timeWindowsGlobal = obj.timeWindowManager.timeWindows;
                obj.updatePriority(startTime);

                assignedPath = candidatePaths(i).nodes;
                assignedWindows = reservedWindows;
                success = true;
                return;
            end
        end

        function mergedTasks = mergePickup(obj, agvObj)
            %MERGEPICKUP Merge pending tasks whose full route lies on the AGV path.
            % Input:
            %   agvObj      - AGV whose current route is inspected.
            % Output:
            %   mergedTasks - Pending tasks merged onto the AGV route.
            pendingTasks = obj.getPendingTasks();
            if isempty(pendingTasks) || isempty(agvObj.path)
                mergedTasks = scheduler.SchedulerClass.emptyTaskArray();
                return;
            end

            mergedTasks = scheduler.SchedulerClass.emptyTaskArray();
            currentNodes = agvObj.path;
            for i = 1:numel(pendingTasks)
                candidateTask = pendingTasks(i);
                if ~scheduler.SchedulerClass.taskFitsPath(currentNodes, candidateTask)
                    continue;
                end

                candidateTask.updateStatus('assigned');
                mergedTasks(end + 1, 1) = candidateTask; %#ok<AGROW>
            end

            if isempty(mergedTasks)
                return;
            end

            if isempty(agvObj.currentTask)
                agvObj.assignTask(mergedTasks);
            elseif isa(agvObj.currentTask, 'task.TaskClass')
                agvObj.assignTask([agvObj.currentTask; mergedTasks]);
            else
                agvObj.assignTask(mergedTasks);
            end

            obj.updatePriority(0.0);
        end

        function orderedTasks = emergencyInsert(obj, taskObj)
            %EMERGENCYINSERT Promote a task to the front of the queue.
            % Input:
            %   taskObj      - Task object to promote.
            % Output:
            %   orderedTasks - Updated ordered pending task queue.
            taskObj.priority = taskObj.priority + obj.emergencyPriorityBoost;
            if ~strcmp(taskObj.status, 'completed')
                taskObj.updateStatus('pending');
            end

            orderedTasks = obj.updatePriority(0.0);
        end
    end

    methods (Access = private)
        function pendingTasks = getPendingTasks(obj)
            pendingMask = arrayfun(@(t) strcmp(t.status, 'pending'), obj.taskList);
            pendingTasks = obj.taskList(pendingMask);
        end

        function [candidatePaths, sourceLabel] = getCandidatePaths(obj, taskObj, agvObj)
            candidatePaths = scheduler.SchedulerClass.emptyCandidatePathArray();
            sourceLabel = 'library';

            libraryEntry = obj.findLibraryEntry(taskObj.id);
            if ~isempty(libraryEntry) && isfield(libraryEntry, 'paths') && ~isempty(libraryEntry.paths)
                candidatePaths = libraryEntry.paths;
                return;
            end

            sourceLabel = 'astar';
            generatedEntry = pathplan.PathLibrary.generateLibrary(obj.map, taskObj, 1, agvObj.speed, agvObj.id);
            if isfield(generatedEntry, 'paths')
                candidatePaths = generatedEntry.paths;
            end
        end

        function libraryEntry = findLibraryEntry(obj, taskId)
            libraryEntry = [];
            if isempty(obj.pathLibraryData)
                return;
            end

            for i = 1:numel(obj.pathLibraryData)
                if isfield(obj.pathLibraryData(i), 'taskId') && obj.pathLibraryData(i).taskId == taskId
                    libraryEntry = obj.pathLibraryData(i);
                    return;
                end
            end
        end
    end

    methods (Static)
        function pathLibraryData = loadDefaultPathLibrary()
            %LOADDEFAULTPATHLIBRARY Load path library data when available.
            pathLibraryData = repmat(struct(), 0, 1);
            libraryPath = fullfile(pwd, 'data', 'path_library.mat');
            if ~isfile(libraryPath)
                return;
            end

            loadedData = load(libraryPath);
            if isfield(loadedData, 'pathLibraryData')
                pathLibraryData = loadedData.pathLibraryData;
            end
        end

        function tf = taskFitsPath(pathNodes, taskObj)
            %TASKFITSPATH Return true when task anchors appear in order on a path.
            requiredNodes = [taskObj.start; taskObj.getWaypointPositions()];
            currentIndex = 1;
            tf = true;

            for i = 1:size(requiredNodes, 1)
                matchFound = false;
                for j = currentIndex:size(pathNodes, 1)
                    if isequal(pathNodes(j, :), requiredNodes(i, :))
                        currentIndex = j + 1;
                        matchFound = true;
                        break;
                    end
                end

                if ~matchFound
                    tf = false;
                    return;
                end
            end
        end

        function tasks = emptyTaskArray()
            %EMPTYTASKARRAY Return an empty task object array.
            tasks = task.TaskClass.empty(0, 1);
        end

        function candidatePaths = emptyCandidatePathArray()
            %EMPTYCANDIDATEPATHARRAY Return an empty candidate path struct array.
            candidatePaths = repmat(struct( ...
                'pathId', 0, ...
                'nodes', zeros(0, 2), ...
                'timeWindows', timewindow.TimeWindowManager.emptyWindowArray(), ...
                'length', 0.0, ...
                'blockedNodesUsed', zeros(0, 2)), 0, 1);
        end
    end
end
