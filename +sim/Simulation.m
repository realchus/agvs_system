classdef Simulation < handle
    %SIMULATION Discrete-time warehouse simulation main loop.
    %   The simulation advances AGVs, assigns pending tasks, resolves
    %   conflicts, handles load/unload timing, and optionally renders the
    %   state in real time.

    properties
        map
        agvPool
        taskList
        scheduler
        timeWindowManager
        visualizer
        config
        currentTime
        eventLog
        serviceStates
        agvTravelDistance
        taskCompletionTimes
        conflictCheckRequired
        parkingPositions
        completedTaskCount
        nextPendingRequestTime
        taskAssignmentRetryTimes
        parkingReservationWindows
    end

    methods
        function obj = Simulation(mapObj, agvPool, taskList, config, schedulerObj, visualizerObj, timeWindowManager)
            %SIMULATION Construct a simulation instance.
            if nargin < 1 || isempty(mapObj)
                mapObj = map.MapClass.createDefaultMap();
            end
            if nargin < 2 || isempty(agvPool) || nargin < 3 || isempty(taskList)
                [defaultAgvPool, defaultTaskList] = sim.Simulation.loadDefaultInputData();
                if nargin < 2 || isempty(agvPool)
                    agvPool = defaultAgvPool;
                end
                if nargin < 3 || isempty(taskList)
                    taskList = defaultTaskList;
                end
            end
            if nargin < 4 || isempty(config)
                config = params();
            end

            if nargin < 7 || isempty(timeWindowManager)
                timeWindowManager = timewindow.TimeWindowManager();
            end

            if nargin < 5 || isempty(schedulerObj)
                pathLibraryData = [];
                if isstruct(config) && isfield(config, 'pathLibraryData')
                    pathLibraryData = config.pathLibraryData;
                end
                schedulerObj = scheduler.SchedulerClass( ...
                    mapObj, agvPool, taskList, pathLibraryData, timeWindowManager);
            end

            if nargin < 6 || isempty(visualizerObj)
                renderEnabled = sim.Simulation.configValue(config, 'enableVisualization', false);
                renderVisible = sim.Simulation.configValue(config, 'visualizerVisible', renderEnabled);
                visualizerObj = sim.Visualizer(renderEnabled, renderVisible);
            end

            obj.map = mapObj;
            obj.agvPool = agvPool;
            obj.taskList = taskList;
            obj.scheduler = schedulerObj;
            obj.timeWindowManager = timeWindowManager;
            obj.visualizer = visualizerObj;
            obj.config = config;
            obj.currentTime = 0.0;
            obj.eventLog = sim.Simulation.emptyEventLog();
            obj.serviceStates = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.agvTravelDistance = containers.Map('KeyType', 'double', 'ValueType', 'double');
            obj.taskCompletionTimes = containers.Map('KeyType', 'double', 'ValueType', 'double');
            obj.conflictCheckRequired = true;
            obj.parkingPositions = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.completedTaskCount = sum(arrayfun(@(t) strcmp(t.status, 'completed'), obj.taskList));
            obj.nextPendingRequestTime = obj.computeNextPendingRequestTime();
            obj.taskAssignmentRetryTimes = containers.Map('KeyType', 'double', 'ValueType', 'double');
            obj.parkingReservationWindows = containers.Map('KeyType', 'double', 'ValueType', 'any');

            for i = 1:numel(obj.agvPool)
                obj.agvTravelDistance(obj.agvPool(i).id) = 0.0;
                obj.parkingPositions(obj.agvPool(i).id) = round(double(obj.agvPool(i).position(:))');
            end

            obj.syncMapOccupancy();
            obj.initializeAssignedTasks();
            obj.updateParkingNodeReservations();
        end

        function results = run(obj)
            %RUN Execute the simulation until time limit or task completion.
            obj.logEvent('simulation_started', 0, 0, 'Simulation started.');

            totalTime = sim.Simulation.configValue(obj.config, 'totalTime', 120.0);
            while obj.currentTime < totalTime && ~obj.isSimulationComplete()
                obj.step();
            end

            obj.logEvent('simulation_finished', 0, 0, 'Simulation finished.');
            results = struct( ...
                'currentTime', obj.currentTime, ...
                'completedTaskCount', obj.completedTaskCount, ...
                'eventLog', obj.eventLog, ...
                'metrics', obj.buildMetrics());
        end

        function step(obj)
            %STEP Advance the simulation by one configured time step.
            dt = sim.Simulation.configValue(obj.config, 'dt', 0.1);

            if obj.assignPendingTasks()
                obj.conflictCheckRequired = true;
            end
            if obj.dispatchIdleReturns()
                obj.conflictCheckRequired = true;
            end
            if obj.conflictCheckRequired
                obj.resolveActiveConflicts();
                obj.conflictCheckRequired = false;
            end

            dynamicObstacles = obj.getDynamicObstacles();
            for i = 1:numel(obj.agvPool)
                moveInfo = obj.advanceAgv(obj.agvPool(i), dt, dynamicObstacles);
                if moveInfo.avoidanceTriggered || moveInfo.conflictDetected || moveInfo.replannedAfterConflict
                    obj.conflictCheckRequired = true;
                end
            end

            obj.updateParkingNodeReservations();
            obj.syncMapOccupancy();
            obj.visualizer.render(obj.map, obj.agvPool, obj.currentTime, dynamicObstacles);
            obj.currentTime = obj.currentTime + dt;
        end
    end

    methods (Access = private)
        function didAssign = assignPendingTasks(obj)
            %ASSIGNPENDINGTASKS Dispatch ready tasks to idle AGVs.
            didAssign = false;
            candidateMask = arrayfun(@(a) isempty(a.currentTask) && any(strcmp(a.state, {'idle', 'returning'})), obj.agvPool);
            candidateAgvs = obj.agvPool(candidateMask);
            if isempty(candidateAgvs)
                return;
            end

            if obj.currentTime + eps < obj.nextPendingRequestTime
                return;
            end

            orderedTasks = obj.scheduler.updatePriority(obj.currentTime);
            skippedTaskIds = [];

            while ~isempty(candidateAgvs)
                nextTask = obj.findAssignableTask(orderedTasks, skippedTaskIds);
                if isempty(nextTask)
                    obj.nextPendingRequestTime = obj.computeNextPendingRequestTime();
                    return;
                end

                rankedIdleIndices = obj.rankIdleAgvs(candidateAgvs, nextTask.start);
                success = false;
                selectedAgv = [];
                selectedIndex = 0;
                assignedPath = zeros(0, 2);
                sourceLabel = '';

                obj.map.registerTaskTarget(nextTask.id, [nextTask.start; nextTask.getWaypointPositions()]);
                for candidateIdx = 1:numel(rankedIdleIndices)
                    selectedIndex = rankedIdleIndices(candidateIdx);
                    selectedAgv = candidateAgvs(selectedIndex);
                    obj.prepareAgvForAssignment(selectedAgv);
                    [success, assignedPath, ~, sourceLabel] = obj.scheduler.assignToAGV(selectedAgv, nextTask, obj.currentTime);
                    if success
                        break;
                    end
                    obj.updateParkingReservationForAgv(selectedAgv);
                end

                if ~success
                    obj.logEvent('task_assignment_deferred', 0, nextTask.id, ...
                        'No idle AGV could reserve this route in the current dispatch pass.');
                    obj.deferTaskRetry(nextTask.id);
                    skippedTaskIds(end + 1) = nextTask.id; %#ok<AGROW>
                    continue;
                end

                nextTask.updateStatus('executing');
                obj.serviceStates(selectedAgv.id) = struct( ...
                    'taskId', nextTask.id, ...
                    'nextWaypointIndex', 1, ...
                    'phase', 'travel');
                didAssign = true;
                obj.clearTaskRetry(nextTask.id);
                obj.logEvent('task_assigned', selectedAgv.id, nextTask.id, ...
                    sprintf('Assigned via %s with %d path nodes.', sourceLabel, size(assignedPath, 1)));
                candidateAgvs(selectedIndex) = [];
            end
            obj.nextPendingRequestTime = obj.computeNextPendingRequestTime();
        end

        function rankedIndices = rankIdleAgvs(obj, idleAgvs, targetPosition)
            %RANKIDLEAGVS Rank idle AGVs by travel distance and workload balance.
            distances = arrayfun(@(a) sum(abs(round(a.position) - targetPosition)), idleAgvs);
            workloads = arrayfun(@(a) obj.travelDistanceFor(a.id), idleAgvs);
            ids = arrayfun(@(a) a.id, idleAgvs);

            balanceWeight = sim.Simulation.configValue(obj.config, 'agvUtilizationBalanceWeight', 20.0);
            if max(workloads) > eps
                workloadPenalty = balanceWeight .* workloads ./ max(workloads);
            else
                workloadPenalty = zeros(size(workloads));
            end

            score = distances + workloadPenalty;
            [~, rankedIndices] = sortrows([score(:), distances(:), workloads(:), ids(:)], [1, 2, 3, 4]);
        end

        function didDispatch = dispatchIdleReturns(obj)
            %DISPATCHIDLERETURNS Send idle AGVs back to their parking cells.
            didDispatch = false;
            if ~sim.Simulation.configValue(obj.config, 'returnToParkingWhenIdle', true)
                return;
            end
            if obj.hasReadyPendingTasks()
                return;
            end

            for i = 1:numel(obj.agvPool)
                agvObj = obj.agvPool(i);
                if ~isempty(agvObj.currentTask) || ~strcmp(agvObj.state, 'idle') || ~isempty(agvObj.path)
                    continue;
                end
                if obj.isAgvParked(agvObj)
                    continue;
                end

                parkingPosition = obj.parkingPositionFor(agvObj);
                currentPosition = round(double(agvObj.position(:))');
                returnTargetId = -agvObj.id;
                returnMap = obj.buildParkingReturnMap(agvObj, currentPosition, parkingPosition);
                returnMap.registerTaskTarget(returnTargetId, [currentPosition; parkingPosition]);
                returnPath = pathplan.AStar(returnMap, currentPosition, parkingPosition, agvObj.id, returnTargetId);
                if isempty(returnPath)
                    obj.logEvent('parking_return_failed', agvObj.id, 0, 'No feasible route to parking point.');
                    continue;
                end

                [returnWindows, conflictInfo] = obj.timeWindowManager.reservePath( ...
                    agvObj.id, returnPath, obj.currentTime, agvObj.speed);
                if ~isempty(conflictInfo)
                    obj.logEvent('parking_return_failed', agvObj.id, 0, 'Parking route reservation conflicted.');
                    continue;
                end

                agvObj.assignPath(returnPath);
                agvObj.setTimeWindows(returnWindows);
                agvObj.updateState('returning');
                didDispatch = true;
                obj.logEvent('parking_return_started', agvObj.id, 0, 'AGV returning to parking point.');
            end
        end

        function nextTask = findAssignableTask(obj, orderedTasks, skippedTaskIds)
            %FINDASSIGNABLETASK Return the first pending task whose request time has arrived.
            if nargin < 3
                skippedTaskIds = [];
            end
            nextTask = [];
            for i = 1:numel(orderedTasks)
                if any(skippedTaskIds == orderedTasks(i).id)
                    continue;
                end
                if strcmp(orderedTasks(i).status, 'pending') && orderedTasks(i).requestTime <= obj.currentTime
                    if obj.isTaskRetryCoolingDown(orderedTasks(i).id)
                        continue;
                    end
                    nextTask = orderedTasks(i);
                    return;
                end
            end
        end

        function resolveActiveConflicts(obj)
            %RESOLVEACTIVECONFLICTS Run pairwise reservation conflict handling.
            waitTimeout = sim.Simulation.configValue(obj.config, 'waitTimeout', 5.0);
            for i = 1:numel(obj.agvPool)
                for j = (i + 1):numel(obj.agvPool)
                    agvA = obj.agvPool(i);
                    agvB = obj.agvPool(j);
                    if isempty(agvA.timeWindows) || isempty(agvB.timeWindows)
                        continue;
                    end

                    [resolved, info] = obj.scheduler.handleConflict(agvA, agvB, obj.currentTime, waitTimeout);
                    if strcmp(info.strategy, 'none')
                        continue;
                    end

                    if resolved
                        obj.logEvent('conflict_resolved', info.adjustedAgvId, 0, info.message);
                    else
                        obj.logEvent('conflict_unresolved', agvA.id, 0, info.message);
                    end
                end
            end
        end

        function moveInfo = advanceAgv(obj, agvObj, dt, dynamicObstacles)
            %ADVANCEAGV Progress one AGV through movement or service states.
            moveInfo = agv.AGVClass.emptyMoveInfo();
            previousPosition = agvObj.position;

            if strcmp(agvObj.state, 'waiting')
                if isempty(agvObj.timeWindows) || obj.currentTime >= agvObj.timeWindows(1).startTime - eps
                    agvObj.updateState('moving');
                else
                    return;
                end
            end

            if strcmp(agvObj.state, 'loading')
                completed = agvObj.load(dt);
                if completed
                    obj.finishLoading(agvObj);
                end
                return;
            end

            if strcmp(agvObj.state, 'unloading')
                completed = agvObj.unload(dt);
                if completed
                    obj.completeTask(agvObj);
                end
                return;
            end

            if isempty(agvObj.currentTask) && strcmp(agvObj.state, 'returning')
                [~, ~, moveInfo] = agvObj.move(dt);
                if strcmp(agvObj.state, 'idle') && obj.isAgvParked(agvObj)
                    obj.timeWindowManager.releasePath(agvObj.id);
                    agvObj.setTimeWindows(timewindow.TimeWindowManager.emptyWindowArray());
                    obj.map.clearTaskTarget(-agvObj.id);
                    obj.updateParkingReservationForAgv(agvObj);
                    obj.logEvent('parking_return_completed', agvObj.id, 0, 'AGV reached parking point.');
                end
                obj.recordTravelDistance(agvObj.id, previousPosition, agvObj.position);
                return;
            end

            if isempty(agvObj.currentTask)
                return;
            end

            if obj.handleWaypointArrival(agvObj)
                return;
            end

            context = struct( ...
                'map', obj.map, ...
                'dynamicObstacles', dynamicObstacles, ...
                'timeWindowManager', obj.timeWindowManager, ...
                'currentTime', obj.currentTime);
            [reachedNode, ~, moveInfo] = agvObj.move(dt, context);

            if moveInfo.avoidanceTriggered
                obj.logEvent('dynamic_avoidance', agvObj.id, obj.primaryTaskId(agvObj), ...
                    sprintf('Applied %s local avoidance.', moveInfo.strategy));
            end
            if moveInfo.replannedAfterConflict
                obj.logEvent('path_replanned', agvObj.id, obj.primaryTaskId(agvObj), ...
                    'Replanned remaining route after avoidance conflict.');
            elseif moveInfo.conflictDetected
                obj.logEvent('avoidance_conflict', agvObj.id, obj.primaryTaskId(agvObj), ...
                    'Avoidance route encountered a reservation conflict.');
            end

            if reachedNode
                obj.handleWaypointArrival(agvObj);
            end

            obj.recordTravelDistance(agvObj.id, previousPosition, agvObj.position);
        end

        function didStartService = handleWaypointArrival(obj, agvObj)
            %HANDLEWAYPOINTARRIVAL Trigger loading or unloading at target nodes.
            didStartService = false;
            if ~isKey(obj.serviceStates, agvObj.id)
                return;
            end

            state = obj.serviceStates(agvObj.id);
            taskObj = obj.primaryTask(agvObj);
            if isempty(taskObj)
                return;
            end

            servicePositions = obj.servicePositionsForTask(taskObj);
            if isempty(servicePositions) || state.nextWaypointIndex > size(servicePositions, 1)
                return;
            end

            targetPosition = servicePositions(state.nextWaypointIndex, :);
            if norm(agvObj.position - targetPosition) > 1e-9
                return;
            end

            if state.nextWaypointIndex < size(servicePositions, 1)
                agvObj.load(0.0);
                state.phase = 'loading';
                obj.serviceStates(agvObj.id) = state;
                obj.logEvent('loading_started', agvObj.id, taskObj.id, ...
                    sprintf('Started loading at waypoint %d.', state.nextWaypointIndex));
            else
                agvObj.unload(0.0);
                state.phase = 'unloading';
                obj.serviceStates(agvObj.id) = state;
                obj.logEvent('unloading_started', agvObj.id, taskObj.id, ...
                    'Started unloading at final waypoint.');
            end
            didStartService = true;
        end

        function finishLoading(obj, agvObj)
            %FINISHLOADING Resume travel after a loading operation completes.
            if ~isKey(obj.serviceStates, agvObj.id)
                return;
            end

            state = obj.serviceStates(agvObj.id);
            state.nextWaypointIndex = state.nextWaypointIndex + 1;
            state.phase = 'travel';
            obj.serviceStates(agvObj.id) = state;
            agvObj.updateState('moving');
            obj.logEvent('loading_completed', agvObj.id, obj.primaryTaskId(agvObj), 'Loading completed.');
        end

        function servicePositions = servicePositionsForTask(~, taskObj)
            %SERVICEPOSITIONSFORTASK Include the task start as the first load point.
            waypointPositions = taskObj.getWaypointPositions();
            if isempty(waypointPositions)
                servicePositions = zeros(0, 2);
                return;
            end

            servicePositions = agv.AGVClass.removeSequentialDuplicates([taskObj.start; waypointPositions]);
        end

        function completeTask(obj, agvObj)
            %COMPLETETASK Finalize one task and release AGV resources.
            taskObj = obj.primaryTask(agvObj);
            if isempty(taskObj)
                return;
            end

            if ~strcmp(taskObj.status, 'completed')
                obj.completedTaskCount = obj.completedTaskCount + 1;
            end
            taskObj.updateStatus('completed');
            obj.taskCompletionTimes(taskObj.id) = obj.currentTime;
            obj.map.clearTaskTarget(taskObj.id);
            obj.timeWindowManager.releasePath(agvObj.id);
            agvObj.setTimeWindows(timewindow.TimeWindowManager.emptyWindowArray());
            agvObj.assignTask([]);
            agvObj.updateState('idle');

            if isKey(obj.serviceStates, agvObj.id)
                remove(obj.serviceStates, agvObj.id);
            end

            obj.logEvent('task_completed', agvObj.id, taskObj.id, 'Task completed.');
        end

        function taskObj = primaryTask(~, agvObj)
            %PRIMARYTASK Return the first task bound to the AGV, if any.
            taskObj = [];
            if isa(agvObj.currentTask, 'task.TaskClass') && ~isempty(agvObj.currentTask)
                taskObj = agvObj.currentTask(1);
            end
        end

        function taskId = primaryTaskId(obj, agvObj)
            %PRIMARYTASKID Return the primary task id or zero when idle.
            taskObj = obj.primaryTask(agvObj);
            if isempty(taskObj)
                taskId = 0;
            else
                taskId = taskObj.id;
            end
        end

        function syncMapOccupancy(obj)
            %SYNCMAPOCCUPANCY Mirror AGV positions onto the dynamic map layer.
            for i = 1:numel(obj.agvPool)
                obj.map.clearAGVOccupancy(obj.agvPool(i).id);
            end

            for i = 1:numel(obj.agvPool)
                agvObj = obj.agvPool(i);
                taskId = obj.primaryTaskId(agvObj);
                if taskId == 0
                    taskId = [];
                end
                gridPosition = round(agvObj.position);
                obj.map.setAGVOccupancy(agvObj.id, gridPosition(1), gridPosition(2), taskId);
            end
        end

        function initializeAssignedTasks(obj)
            %INITIALIZEASSIGNEDTASKS Restore service state for preassigned AGVs.
            for i = 1:numel(obj.agvPool)
                agvObj = obj.agvPool(i);
                taskObj = obj.primaryTask(agvObj);
                if isempty(taskObj)
                    continue;
                end

                obj.map.registerTaskTarget(taskObj.id, [taskObj.start; taskObj.getWaypointPositions()]);
                obj.serviceStates(agvObj.id) = struct( ...
                    'taskId', taskObj.id, ...
                    'nextWaypointIndex', 1, ...
                    'phase', 'travel');
            end
        end

        function dynamicObstacles = getDynamicObstacles(obj)
            %GETDYNAMICOBSTACLES Return currently active dynamic obstacle positions.
            schedule = sim.Simulation.configValue(obj.config, 'dynamicObstacleSchedule', repmat(struct(), 0, 1));
            dynamicObstacles = zeros(0, 2);
            dt = sim.Simulation.configValue(obj.config, 'dt', 0.1);

            for i = 1:numel(schedule)
                entry = schedule(i);
                if isfield(entry, 'startTime')
                    startTime = entry.startTime;
                elseif isfield(entry, 'time')
                    startTime = entry.time;
                else
                    startTime = 0.0;
                end

                if isfield(entry, 'endTime')
                    endTime = entry.endTime;
                else
                    endTime = startTime + dt;
                end

                if obj.currentTime >= startTime && obj.currentTime < endTime && isfield(entry, 'positions')
                    dynamicObstacles = [dynamicObstacles; double(entry.positions)]; %#ok<AGROW>
                end
            end
        end

        function tf = allTasksCompleted(obj)
            %ALLTASKSCOMPLETED Return true when every task is marked completed.
            tf = obj.completedTaskCount >= numel(obj.taskList);
        end

        function tf = isSimulationComplete(obj)
            %ISSIMULATIONCOMPLETE Include optional post-task return-to-parking.
            if ~obj.allTasksCompleted()
                tf = false;
                return;
            end

            if ~sim.Simulation.configValue(obj.config, 'returnToParkingWhenIdle', true)
                tf = true;
                return;
            end

            tf = obj.allAgvsParked();
        end

        function tf = allAgvsParked(obj)
            %ALLAGVSPARKED Return true when every AGV is idle at its parking cell.
            tf = true;
            for i = 1:numel(obj.agvPool)
                agvObj = obj.agvPool(i);
                if ~isempty(agvObj.currentTask) || ~strcmp(agvObj.state, 'idle') || ~obj.isAgvParked(agvObj)
                    tf = false;
                    return;
                end
            end
        end

        function tf = isAgvParked(obj, agvObj)
            %ISAGVPARKED Compare current position with the stored parking point.
            tf = isequal(round(double(agvObj.position(:))'), obj.parkingPositionFor(agvObj));
        end

        function tf = hasReadyPendingTasks(obj)
            %HASREADYPENDINGTASKS Return true when a pending task can be dispatched now.
            tf = false;
            for i = 1:numel(obj.taskList)
                if strcmp(obj.taskList(i).status, 'pending') && obj.taskList(i).requestTime <= obj.currentTime
                    tf = true;
                    return;
                end
            end
        end

        function prepareAgvForAssignment(obj, agvObj)
            %PREPAREAGVFORASSIGNMENT Clear return/parking reservations before dispatch.
            obj.releaseParkingReservation(agvObj.id);
            if strcmp(agvObj.state, 'returning')
                obj.timeWindowManager.releasePath(agvObj.id);
                obj.map.clearTaskTarget(-agvObj.id);
                agvObj.assignPath([]);
                agvObj.setTimeWindows(timewindow.TimeWindowManager.emptyWindowArray());
                agvObj.updateState('idle');
                obj.logEvent('parking_return_interrupted', agvObj.id, 0, ...
                    'Interrupted parking return for a ready task.');
            end
        end

        function tf = isTaskRetryCoolingDown(obj, taskId)
            %ISTASKRETRYCOOLINGDOWN Return true when a deferred task should wait.
            tf = isKey(obj.taskAssignmentRetryTimes, taskId) && ...
                obj.currentTime + eps < obj.taskAssignmentRetryTimes(taskId);
        end

        function deferTaskRetry(obj, taskId)
            %DEFERTASKRETRY Avoid repeating the same failed route search every step.
            retryDelay = sim.Simulation.configValue(obj.config, 'assignmentRetryDelay', 10.0);
            obj.taskAssignmentRetryTimes(taskId) = obj.currentTime + retryDelay;
        end

        function clearTaskRetry(obj, taskId)
            %CLEARTASKRETRY Remove retry throttling once a task is assigned.
            if isKey(obj.taskAssignmentRetryTimes, taskId)
                remove(obj.taskAssignmentRetryTimes, taskId);
            end
        end

        function returnMap = buildParkingReturnMap(obj, agvObj, currentPosition, parkingPosition)
            %BUILDPARKINGRETURNMAP Keep other parking cells out of return routes.
            blockedGrid = obj.map.baseGrid;
            for i = 1:numel(obj.agvPool)
                otherAgv = obj.agvPool(i);
                if otherAgv.id == agvObj.id
                    continue;
                end
                otherParking = obj.parkingPositionFor(otherAgv);
                if isequal(otherParking, currentPosition) || isequal(otherParking, parkingPosition)
                    continue;
                end
                blockedGrid(otherParking(1), otherParking(2)) = 3;
            end

            returnMap = map.MapClass(blockedGrid, obj.map.colors);
            for i = 1:numel(obj.agvPool)
                otherAgv = obj.agvPool(i);
                taskId = obj.primaryTaskId(otherAgv);
                if taskId == 0
                    taskId = [];
                end
                gridPosition = round(otherAgv.position);
                if returnMap.isPassable(gridPosition(1), gridPosition(2), otherAgv.id, taskId)
                    returnMap.setAGVOccupancy(otherAgv.id, gridPosition(1), gridPosition(2), taskId);
                end
            end
        end

        function updateParkingNodeReservations(obj)
            %UPDATEPARKINGNODERESERVATIONS Keep idle parked AGVs reserved.
            for i = 1:numel(obj.agvPool)
                obj.updateParkingReservationForAgv(obj.agvPool(i));
            end
        end

        function updateParkingReservationForAgv(obj, agvObj)
            %UPDATEPARKINGRESERVATIONFORAGV Reserve/release one parked AGV node.
            shouldReserve = isempty(agvObj.currentTask) && strcmp(agvObj.state, 'idle') && ...
                isempty(agvObj.path) && obj.isAgvParked(agvObj);
            if shouldReserve
                obj.reserveParkingReservation(agvObj);
            else
                obj.releaseParkingReservation(agvObj.id);
            end
        end

        function reserveParkingReservation(obj, agvObj)
            %RESERVEPARKINGRESERVATION Register a long node window for parking.
            if isKey(obj.parkingReservationWindows, agvObj.id)
                return;
            end

            parkingPosition = obj.parkingPositionFor(agvObj);
            totalTime = sim.Simulation.configValue(obj.config, 'totalTime', 120.0);
            endTime = max(totalTime + 1.0, obj.currentTime + 1.0);
            parkingWindow = timewindow.TimeWindowManager.buildNodeWindow( ...
                parkingPosition, obj.currentTime, endTime, agvObj.id);
            [hasConflict, ~] = obj.timeWindowManager.detectConflict(parkingWindow);
            if hasConflict
                obj.logEvent('parking_reservation_failed', agvObj.id, 0, ...
                    'Parking node reservation conflicted.');
                return;
            end

            obj.timeWindowManager.timeWindows(end + 1, 1) = parkingWindow;
            obj.parkingReservationWindows(agvObj.id) = parkingWindow;
        end

        function releaseParkingReservation(obj, agvId)
            %RELEASEPARKINGRESERVATION Remove the stored parking node window.
            if ~isKey(obj.parkingReservationWindows, agvId)
                return;
            end

            parkingWindow = obj.parkingReservationWindows(agvId);
            keepMask = true(numel(obj.timeWindowManager.timeWindows), 1);
            for i = 1:numel(obj.timeWindowManager.timeWindows)
                if sim.Simulation.isSameWindow(obj.timeWindowManager.timeWindows(i), parkingWindow)
                    keepMask(i) = false;
                end
            end
            obj.timeWindowManager.timeWindows = obj.timeWindowManager.timeWindows(keepMask);
            remove(obj.parkingReservationWindows, agvId);
        end

        function parkingPosition = parkingPositionFor(obj, agvObj)
            %PARKINGPOSITIONFOR Return the AGV's initial parking coordinate.
            if isKey(obj.parkingPositions, agvObj.id)
                parkingPosition = obj.parkingPositions(agvObj.id);
            else
                parkingPosition = round(double(agvObj.position(:))');
            end
        end

        function logEvent(obj, type, agvId, taskId, message)
            %LOGEVENT Append a simulation event entry to the in-memory log.
            entry = struct( ...
                'time', obj.currentTime, ...
                'type', char(string(type)), ...
                'agvId', double(agvId), ...
                'taskId', double(taskId), ...
                'message', char(string(message)));
            obj.eventLog(end + 1, 1) = entry; %#ok<AGROW>
        end

        function recordTravelDistance(obj, agvId, previousPosition, currentPosition)
            %RECORDTRAVELDISTANCE Accumulate AGV travel distance for reporting.
            if ~isKey(obj.agvTravelDistance, agvId)
                obj.agvTravelDistance(agvId) = 0.0;
            end

            obj.agvTravelDistance(agvId) = obj.agvTravelDistance(agvId) + ...
                norm(double(currentPosition) - double(previousPosition));
        end

        function distance = travelDistanceFor(obj, agvId)
            %TRAVELDISTANCEFOR Return accumulated travel distance for one AGV.
            if isKey(obj.agvTravelDistance, agvId)
                distance = obj.agvTravelDistance(agvId);
            else
                distance = 0.0;
            end
        end

        function metrics = buildMetrics(obj)
            %BUILDMETRICS Export scenario-level summary metrics.
            metrics = struct();
            metrics.totalTime = obj.currentTime;
            metrics.completedTaskCount = obj.completedTaskCount;
            metrics.collisionCount = sim.Simulation.countEvents(obj.eventLog, 'collision_detected');
            metrics.conflictResolutionCount = sim.Simulation.countEvents(obj.eventLog, 'conflict_resolved');
            metrics.replanCount = sim.Simulation.countEvents(obj.eventLog, 'path_replanned');
            metrics.avoidanceCount = sim.Simulation.countEvents(obj.eventLog, 'dynamic_avoidance');
            metrics.taskAssignmentFailureCount = sim.Simulation.countEvents(obj.eventLog, 'task_assignment_failed');
            metrics.agvDistances = obj.exportAgvDistances();
            metrics.taskCompletionTimes = obj.exportTaskCompletionTimes();
        end

        function agvDistances = exportAgvDistances(obj)
            %EXPORTAGVDISTANCES Convert AGV distance counters into a struct array.
            keysList = sort(cell2mat(obj.agvTravelDistance.keys));
            agvDistances = repmat(struct('agvId', 0, 'distance', 0.0), numel(keysList), 1);
            for i = 1:numel(keysList)
                agvDistances(i, 1) = struct( ...
                    'agvId', keysList(i), ...
                    'distance', obj.agvTravelDistance(keysList(i)));
            end
        end

        function completionTimes = exportTaskCompletionTimes(obj)
            %EXPORTTASKCOMPLETIONTIMES Export task finish timestamps for reporting.
            completionTimes = repmat(struct('taskId', 0, 'completionTime', 0.0), 0, 1);
            if isempty(obj.taskCompletionTimes)
                return;
            end

            keysList = sort(cell2mat(obj.taskCompletionTimes.keys));
            completionTimes = repmat(struct('taskId', 0, 'completionTime', 0.0), numel(keysList), 1);
            for i = 1:numel(keysList)
                completionTimes(i, 1) = struct( ...
                    'taskId', keysList(i), ...
                    'completionTime', obj.taskCompletionTimes(keysList(i)));
            end
        end

        function nextRequestTime = computeNextPendingRequestTime(obj)
            %COMPUTENEXTPENDINGREQUESTTIME Return the next unreleased pending task time.
            nextRequestTime = inf;
            for i = 1:numel(obj.taskList)
                if strcmp(obj.taskList(i).status, 'pending')
                    nextRequestTime = min(nextRequestTime, obj.taskList(i).requestTime);
                end
            end
        end
    end

    methods (Static)
        function obj = fromDefaults(config)
            %FROMDEFAULTS Build a simulation using default project data.
            if nargin < 1 || isempty(config)
                config = params();
            end

            [agvPool, taskList] = sim.Simulation.loadDefaultInputData();
            obj = sim.Simulation( ...
                map.MapClass.createDefaultMap(), ...
                agvPool, ...
                taskList, ...
                config);
        end

        function [agvPool, taskList] = loadDefaultInputData()
            %LOADDEFAULTINPUTDATA Prefer user-editable JSON files under input/.
            inputDir = fullfile(pwd, 'input');
            agvJsonPath = fullfile(inputDir, 'agv_pool.json');
            taskJsonPath = fullfile(inputDir, 'task_list.json');

            if isfile(agvJsonPath)
                agvPool = agv.AGVClass.loadPool(agvJsonPath);
            else
                agvPool = agv.AGVClass.createDefaultPool();
            end

            if isfile(taskJsonPath)
                taskList = task.TaskParser(taskJsonPath);
            else
                taskList = task.TaskParser(fullfile(pwd, 'data', 'task_list.mat'));
            end
        end

        function value = configValue(config, fieldName, defaultValue)
            %CONFIGVALUE Read a config field or return a default.
            if isstruct(config) && isfield(config, fieldName) && ~isempty(config.(fieldName))
                value = config.(fieldName);
            else
                value = defaultValue;
            end
        end

        function eventLog = emptyEventLog()
            %EMPTYEVENTLOG Return an empty event-log struct array.
            eventLog = repmat(struct( ...
                'time', 0.0, ...
                'type', '', ...
                'agvId', 0, ...
                'taskId', 0, ...
                'message', ''), 0, 1);
        end

        function count = countEvents(eventLog, type)
            %COUNTEVENTS Count logged events of a given type.
            if isempty(eventLog)
                count = 0;
                return;
            end

            count = sum(strcmp({eventLog.type}, char(string(type))));
        end

        function tf = isSameWindow(windowA, windowB)
            %ISSAMEWINDOW Compare time-window resource, owner, and timing.
            windowA = timewindow.TimeWindowManager.normalizeWindow(windowA);
            windowB = timewindow.TimeWindowManager.normalizeWindow(windowB);
            tf = strcmp(windowA.windowType, windowB.windowType) && ...
                isequal(windowA.edgeIndex, windowB.edgeIndex) && ...
                isequal(windowA.nodeIndex, windowB.nodeIndex) && ...
                windowA.agvId == windowB.agvId && ...
                abs(windowA.startTime - windowB.startTime) <= eps && ...
                abs(windowA.endTime - windowB.endTime) <= eps;
        end
    end
end
