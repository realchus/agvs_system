classdef SchedulerClass < handle
    %SCHEDULERCLASS Task scheduler for AGV assignment and queue management.
    %   The scheduler ranks pending tasks, assigns feasible routes to AGVs,
    %   and resolves path conflicts using wait-or-replan policies.

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
            if nargin < 4
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
            %   agvObj         - AGV object to receive the task.
            %   taskObj        - Task object to assign.
            %   startTime      - Optional path start time.
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

            obj.map.registerTaskTarget(taskObj.id, [taskObj.start; taskObj.getWaypointPositions()]);
            [candidatePaths, sourceLabel] = obj.getCandidatePaths(taskObj, agvObj);
            if isempty(candidatePaths)
                sourceLabel = 'astar';
                return;
            end

            for i = 1:numel(candidatePaths)
                executablePath = obj.buildExecutablePath(agvObj, taskObj, candidatePaths(i).nodes);
                if isempty(executablePath)
                    continue;
                end

                [reservedWindows, conflictInfo] = obj.timeWindowManager.reservePath( ...
                    agvObj.id, executablePath, startTime, agvObj.speed);
                if ~isempty(conflictInfo)
                    continue;
                end

                taskObj.updateStatus('assigned');
                agvObj.assignTask(taskObj);
                agvObj.assignPath(executablePath);
                agvObj.setTimeWindows(reservedWindows);
                agvObj.updateState('moving');

                obj.timeWindowsGlobal = obj.timeWindowManager.timeWindows;
                obj.updatePriority(startTime);

                assignedPath = executablePath;
                assignedWindows = reservedWindows;
                success = true;
                return;
            end
        end

        function [resolved, resolutionInfo] = handleConflict(obj, agvA, agvB, currentTime, waitTimeout)
            %HANDLECONFLICT Resolve a reserved-path conflict between two AGVs.
            % Inputs:
            %   agvA        - First AGV involved in the conflict.
            %   agvB        - Second AGV involved in the conflict.
            %   currentTime - Scheduler time used for delay/replan decisions.
            %   waitTimeout - Maximum allowed delayed-start time in seconds.
            % Outputs:
            %   resolved       - True when the conflict is resolved.
            %   resolutionInfo - Struct describing the chosen strategy.
            if nargin < 4 || isempty(currentTime)
                currentTime = 0.0;
            end
            if nargin < 5 || isempty(waitTimeout)
                waitTimeout = 5.0;
            end

            resolutionInfo = scheduler.SchedulerClass.emptyResolutionInfo();
            [hasConflict, conflictInfo] = obj.findConflictBetweenAGVs(agvA, agvB);
            if ~hasConflict
                resolved = true;
                resolutionInfo.strategy = 'none';
                resolutionInfo.message = 'No conflict detected.';
                return;
            end

            [keeperAgv, adjustedAgv] = obj.selectConflictPriority(agvA, agvB);
            adjustedTask = obj.getPrimaryTaskFromAGV(adjustedAgv);
            keeperWindow = obj.getWindowForAgv(conflictInfo, keeperAgv.id);

            resolutionInfo.conflict = conflictInfo;
            resolutionInfo.keptAgvId = keeperAgv.id;
            resolutionInfo.adjustedAgvId = adjustedAgv.id;

            originalWindows = adjustedAgv.timeWindows;
            originalState = adjustedAgv.state;
            if isempty(originalWindows)
                originalStartTime = currentTime;
            else
                originalStartTime = max(currentTime, originalWindows(1).startTime);
            end

            obj.timeWindowManager.releasePath(adjustedAgv.id);

            delayedStartTime = max(originalStartTime, keeperWindow.endTime);
            delayAmount = delayedStartTime - originalStartTime;

            if delayAmount <= waitTimeout
                [waitWindows, waitConflict] = obj.timeWindowManager.reservePath( ...
                    adjustedAgv.id, adjustedAgv.path, delayedStartTime, adjustedAgv.speed);
                if isempty(waitConflict)
                    adjustedAgv.assignPath(adjustedAgv.path);
                    adjustedAgv.setTimeWindows(waitWindows);
                    adjustedAgv.updateState('waiting');

                    obj.timeWindowsGlobal = obj.timeWindowManager.timeWindows;
                    obj.updatePriority(currentTime);

                    resolved = true;
                    resolutionInfo.strategy = 'wait';
                    resolutionInfo.delay = delayAmount;
                    resolutionInfo.startTime = delayedStartTime;
                    resolutionInfo.newPath = adjustedAgv.path;
                    resolutionInfo.newWindows = waitWindows;
                    resolutionInfo.message = sprintf( ...
                        'AGV %d delayed by %.2f seconds to avoid AGV %d.', ...
                        adjustedAgv.id, delayAmount, keeperAgv.id);
                    return;
                end
            end

            [resolved, replannedPath, replannedWindows, sourceLabel] = ...
                obj.tryReplanStrategy(adjustedAgv, adjustedTask, currentTime, conflictInfo);

            if resolved
                adjustedAgv.assignPath(replannedPath);
                adjustedAgv.setTimeWindows(replannedWindows);
                adjustedAgv.updateState('moving');

                obj.timeWindowsGlobal = obj.timeWindowManager.timeWindows;
                obj.updatePriority(currentTime);

                resolutionInfo.strategy = 'replan';
                resolutionInfo.delay = delayAmount;
                resolutionInfo.startTime = currentTime;
                resolutionInfo.sourceLabel = sourceLabel;
                resolutionInfo.newPath = replannedPath;
                resolutionInfo.newWindows = replannedWindows;
                resolutionInfo.message = sprintf( ...
                    'AGV %d replanned to avoid AGV %d.', adjustedAgv.id, keeperAgv.id);
                return;
            end

            obj.timeWindowManager.timeWindows = [obj.timeWindowManager.timeWindows; originalWindows];
            adjustedAgv.setTimeWindows(originalWindows);
            adjustedAgv.updateState(originalState);
            obj.timeWindowsGlobal = obj.timeWindowManager.timeWindows;

            resolved = false;
            resolutionInfo.strategy = 'unresolved';
            resolutionInfo.delay = delayAmount;
            resolutionInfo.message = sprintf( ...
                'Conflict between AGV %d and AGV %d could not be resolved automatically.', ...
                agvA.id, agvB.id);
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
            %GETPENDINGTASKS Return all tasks that are still waiting for dispatch.
            pendingMask = arrayfun(@(t) strcmp(t.status, 'pending'), obj.taskList);
            pendingTasks = obj.taskList(pendingMask);
        end

        function [candidatePaths, sourceLabel] = getCandidatePaths(obj, taskObj, agvObj)
            %GETCANDIDATEPATHS Prefer library routes and fall back to on-demand A*.
            candidatePaths = scheduler.SchedulerClass.emptyCandidatePathArray();
            sourceLabel = 'library';
            if isempty(taskObj)
                return;
            end

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
            %FINDLIBRARYENTRY Look up a path-library record by task id.
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

        function [hasConflict, conflictInfo] = findConflictBetweenAGVs(obj, agvA, agvB)
            %FINDCONFLICTBETWEENAGVS Search for the first reservation conflict.
            hasConflict = false;
            conflictInfo = scheduler.SchedulerClass.emptyConflictInfo();

            if isempty(agvA.timeWindows) || isempty(agvB.timeWindows)
                return;
            end

            agvBWindowIndex = timewindow.TimeWindowManager.buildWindowIndex(agvB.timeWindows);
            for i = 1:numel(agvA.timeWindows)
                [hasConflict, rawConflict] = obj.timeWindowManager.detectConflict( ...
                    agvA.timeWindows(i), [], agvBWindowIndex);
                if ~hasConflict
                    continue;
                end

                conflictInfo = rawConflict;
                conflictInfo.agvAWindow = agvA.timeWindows(i);
                conflictInfo.agvBWindow = rawConflict.existingWindow;
                return;
            end
        end

        function [keeperAgv, adjustedAgv] = selectConflictPriority(obj, agvA, agvB)
            %SELECTCONFLICTPRIORITY Choose which AGV keeps its current plan.
            comparison = obj.compareAgvPriority(agvA, agvB);
            if comparison >= 0
                keeperAgv = agvA;
                adjustedAgv = agvB;
            else
                keeperAgv = agvB;
                adjustedAgv = agvA;
            end
        end

        function comparison = compareAgvPriority(obj, agvA, agvB)
            %COMPAREAGVPRIORITY Compare AGVs using task priority and FIFO order.
            [priorityA, requestTimeA] = obj.getAgvPriorityInfo(agvA);
            [priorityB, requestTimeB] = obj.getAgvPriorityInfo(agvB);

            if priorityA ~= priorityB
                comparison = sign(priorityA - priorityB);
                return;
            end

            if requestTimeA ~= requestTimeB
                comparison = sign(requestTimeB - requestTimeA);
                return;
            end

            comparison = sign(agvB.id - agvA.id);
        end

        function [priorityValue, requestTimeValue] = getAgvPriorityInfo(~, agvObj)
            %GETAGVPRIORITYINFO Extract comparison fields from an AGV payload.
            if ~isa(agvObj.currentTask, 'task.TaskClass') || isempty(agvObj.currentTask)
                priorityValue = 0.0;
                requestTimeValue = inf;
                return;
            end

            priorityValue = max(arrayfun(@(t) t.priority, agvObj.currentTask));
            requestTimeValue = min(arrayfun(@(t) t.requestTime, agvObj.currentTask));
        end

        function taskObj = getPrimaryTaskFromAGV(~, agvObj)
            %GETPRIMARYTASKFROMAGV Return the lead task bound to an AGV.
            if isa(agvObj.currentTask, 'task.TaskClass') && ~isempty(agvObj.currentTask)
                taskObj = agvObj.currentTask(1);
            else
                taskObj = [];
            end
        end

        function window = getWindowForAgv(~, conflictInfo, agvId)
            %GETWINDOWFORAGV Extract the conflict window that belongs to one AGV.
            if conflictInfo.newWindow.agvId == agvId
                window = conflictInfo.newWindow;
            elseif ~isempty(conflictInfo.existingWindow) && conflictInfo.existingWindow.agvId == agvId
                window = conflictInfo.existingWindow;
            elseif conflictInfo.agvAWindow.agvId == agvId
                window = conflictInfo.agvAWindow;
            else
                window = conflictInfo.agvBWindow;
            end
        end

        function [success, replannedPath, replannedWindows, sourceLabel] = ...
                tryReplanStrategy(obj, agvObj, taskObj, currentTime, conflictInfo)
            %TRYREPLANSTRATEGY Attempt alternate library routes, then conflict-aware A*.
            success = false;
            replannedPath = zeros(0, 2);
            replannedWindows = timewindow.TimeWindowManager.emptyWindowArray();
            sourceLabel = '';

            if isempty(taskObj)
                return;
            end

            [candidatePaths, sourceLabel] = obj.getCandidatePaths(taskObj, agvObj);
            for i = 1:numel(candidatePaths)
                candidateNodes = candidatePaths(i).nodes;
                if isempty(candidateNodes)
                    continue;
                end
                if ~scheduler.SchedulerClass.pathStartsAtPosition(candidateNodes, agvObj.position)
                    continue;
                end
                if scheduler.SchedulerClass.arePathsEqual(candidateNodes, agvObj.path)
                    continue;
                end

                [reservedWindows, conflictDetails] = obj.timeWindowManager.reservePath( ...
                    agvObj.id, candidateNodes, currentTime, agvObj.speed);
                if ~isempty(conflictDetails)
                    continue;
                end

                success = true;
                replannedPath = candidateNodes;
                replannedWindows = reservedWindows;
                return;
            end

            sourceLabel = 'astar';
            fallbackPath = obj.buildConflictAwareRoute(agvObj, taskObj, conflictInfo);
            if isempty(fallbackPath) || scheduler.SchedulerClass.arePathsEqual(fallbackPath, agvObj.path)
                return;
            end

            [reservedWindows, conflictDetails] = obj.timeWindowManager.reservePath( ...
                agvObj.id, fallbackPath, currentTime, agvObj.speed);
            if ~isempty(conflictDetails)
                return;
            end

            success = true;
            replannedPath = fallbackPath;
            replannedWindows = reservedWindows;
        end

        function route = buildConflictAwareRoute(obj, agvObj, taskObj, conflictInfo)
            %BUILDCONFLICTAWAREROUTE Replan a route while blocking conflicting nodes.
            route = zeros(0, 2);
            currentNode = round(double(agvObj.position(:))');
            waypointPositions = taskObj.getWaypointPositions();
            anchors = [currentNode; waypointPositions];
            if size(anchors, 1) < 2
                route = currentNode;
                return;
            end

            workingMap = scheduler.SchedulerClass.cloneMap(obj.map);
            protectedNodes = scheduler.SchedulerClass.uniqueRows(anchors);
            blockedNodes = scheduler.SchedulerClass.conflictNodes(conflictInfo);
            scheduler.SchedulerClass.applyBlockedNodesToMap(workingMap, blockedNodes, protectedNodes);
            workingMap.registerTaskTarget(taskObj.id, protectedNodes);

            for segmentIdx = 1:(size(anchors, 1) - 1)
                segmentPath = pathplan.AStar(workingMap, anchors(segmentIdx, :), ...
                    anchors(segmentIdx + 1, :), agvObj.id, taskObj.id);
                if isempty(segmentPath)
                    route = zeros(0, 2);
                    return;
                end

                if isempty(route)
                    route = segmentPath;
                else
                    route = [route; segmentPath(2:end, :)]; %#ok<AGROW>
                end
            end
        end

        function executablePath = buildExecutablePath(obj, agvObj, taskObj, taskRoute)
            %BUILDEXECUTABLEPATH Prepend a reposition leg when an AGV is off-route.
            executablePath = zeros(0, 2);
            if isempty(taskRoute)
                return;
            end

            currentNode = round(double(agvObj.position(:))');
            routeStart = double(taskRoute(1, :));
            if isequal(currentNode, routeStart)
                executablePath = double(taskRoute);
                return;
            end

            repositionPath = pathplan.AStar(obj.map, currentNode, routeStart, agvObj.id, taskObj.id);
            if isempty(repositionPath)
                return;
            end

            executablePath = [double(repositionPath); double(taskRoute(2:end, :))];
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

        function info = emptyConflictInfo()
            %EMPTYCONFLICTINFO Return an empty conflict info struct.
            info = struct( ...
                'type', '', ...
                'message', '', ...
                'newWindow', timewindow.TimeWindowManager.emptyWindowArray(), ...
                'existingWindow', timewindow.TimeWindowManager.emptyWindowArray(), ...
                'agvAWindow', timewindow.TimeWindowManager.emptyWindowArray(), ...
                'agvBWindow', timewindow.TimeWindowManager.emptyWindowArray());
        end

        function info = emptyResolutionInfo()
            %EMPTYRESOLUTIONINFO Return an empty conflict-resolution struct.
            info = struct( ...
                'strategy', '', ...
                'message', '', ...
                'keptAgvId', 0, ...
                'adjustedAgvId', 0, ...
                'delay', 0.0, ...
                'startTime', 0.0, ...
                'sourceLabel', '', ...
                'newPath', zeros(0, 2), ...
                'newWindows', timewindow.TimeWindowManager.emptyWindowArray(), ...
                'conflict', scheduler.SchedulerClass.emptyConflictInfo());
        end

        function tf = pathStartsAtPosition(pathNodes, position)
            %PATHSTARTSATPOSITION Return true when a path begins at a position.
            tf = ~isempty(pathNodes) && isequal(double(pathNodes(1, :)), round(double(position(:))'));
        end

        function tf = arePathsEqual(pathA, pathB)
            %AREPATHSEQUAL Compare two numeric paths.
            tf = isequal(double(pathA), double(pathB));
        end

        function workingMap = cloneMap(mapObj)
            %CLONEMAP Clone a map including occupancy and task target metadata.
            config = mapObj.toStruct();
            workingMap = map.MapClass(config.baseGrid, config.colors);

            for i = 1:numel(config.occupancy)
                workingMap.setAGVOccupancy(config.occupancy(i).id, ...
                    config.occupancy(i).position(1), config.occupancy(i).position(2));
            end

            for i = 1:numel(config.taskTargets)
                workingMap.registerTaskTarget(config.taskTargets(i).id, config.taskTargets(i).positions);
            end
        end

        function applyBlockedNodesToMap(mapObj, blockedNodes, protectedNodes)
            %APPLYBLOCKEDNODESTOMAP Mark selected nodes as temporarily blocked.
            for i = 1:size(blockedNodes, 1)
                node = blockedNodes(i, :);
                if any(all(protectedNodes == node, 2))
                    continue;
                end
                if node(1) < 1 || node(1) > size(mapObj.baseGrid, 1) || ...
                        node(2) < 1 || node(2) > size(mapObj.baseGrid, 2)
                    continue;
                end

                occupiedByOther = mapObj.grid(node(1), node(2)) == 2;
                mapObj.baseGrid(node(1), node(2)) = 3;
                if ~occupiedByOther
                    mapObj.grid(node(1), node(2)) = 3;
                end
            end
        end

        function nodes = conflictNodes(conflictInfo)
            %CONFLICTNODES Return unique nodes involved in a conflicting edge.
            windows = timewindow.TimeWindowManager.emptyWindowArray();
            if ~isempty(conflictInfo.newWindow)
                windows = [windows; conflictInfo.newWindow];
            end
            if ~isempty(conflictInfo.existingWindow)
                windows = [windows; conflictInfo.existingWindow];
            end

            nodes = zeros(0, 2);
            for i = 1:numel(windows)
                nodes = [nodes; windows(i).edgeIndex(1:2); windows(i).edgeIndex(3:4)]; %#ok<AGROW>
            end
            nodes = scheduler.SchedulerClass.uniqueRows(nodes);
        end

        function matrix = uniqueRows(matrix)
            %UNIQUEROWS Return unique rows while preserving order.
            if isempty(matrix)
                return;
            end

            [~, uniqueIdx] = unique(double(matrix), 'rows', 'stable');
            matrix = double(matrix(sort(uniqueIdx), :));
        end
    end
end
