classdef AGVClass < handle
    %AGVCLASS AGV model with movement, task, and load/unload state handling.
    %   The class stores AGV runtime status and can advance along a grid
    %   path using a fixed speed model. When dynamic obstacles appear ahead,
    %   the AGV can invoke a lightweight DWA local planner for avoidance.

    properties
        id
        position
        state
        currentTask
        speed
        path
        pathIndex
        timeWindows
        loadDuration
        unloadDuration
        operationRemainingTime
        isLoaded
        dwaPlanner
        dynamicObstacleLookahead
        invalidatedTimeWindows
        lastMoveInfo
    end

    methods
        function obj = AGVClass(id, startPos, speed)
            %AGVCLASS Construct an AGV object.
            % Inputs:
            %   id       - AGV identifier.
            %   startPos - 1-by-2 starting coordinate [row, col].
            %   speed    - Travel speed in grid units per second.
            if nargin < 1
                error('AGVClass:MissingId', 'AGV id is required.');
            end
            if nargin < 2 || isempty(startPos)
                startPos = [1, 1];
            end
            if nargin < 3 || isempty(speed)
                speed = 1.0;
            end

            validateattributes(id, {'numeric'}, {'scalar', 'integer', 'positive'});
            validateattributes(startPos, {'numeric'}, {'vector', 'numel', 2, 'finite', 'positive'});
            validateattributes(speed, {'numeric'}, {'scalar', 'positive', 'finite'});

            obj.id = double(id);
            obj.position = double(startPos(:))';
            obj.state = 'idle';
            obj.currentTask = [];
            obj.speed = double(speed);
            obj.path = zeros(0, 2);
            obj.pathIndex = 1;
            obj.timeWindows = [];
            obj.loadDuration = 5.0;
            obj.unloadDuration = 5.0;
            obj.operationRemainingTime = 0.0;
            obj.isLoaded = false;
            obj.dwaPlanner = pathplan.DWAClass(struct('maxSpeed', obj.speed));
            obj.dynamicObstacleLookahead = 2;
            obj.invalidatedTimeWindows = timewindow.TimeWindowManager.emptyWindowArray();
            obj.lastMoveInfo = agv.AGVClass.emptyMoveInfo();
        end

        function assignTask(obj, task)
            %ASSIGNTASK Attach a task to the AGV.
            % Input:
            %   task - Task object or task metadata.
            obj.currentTask = task;
        end

        function assignPath(obj, path)
            %ASSIGNPATH Set a new path for the AGV.
            % Input:
            %   path - N-by-2 path coordinates.
            if isempty(path)
                obj.path = zeros(0, 2);
                obj.pathIndex = 1;
                if any(strcmp(obj.state, {'moving', 'avoiding'}))
                    obj.state = 'idle';
                end
                return;
            end

            validateattributes(path, {'numeric'}, {'2d', 'ncols', 2, 'finite'});

            obj.path = double(path);
            if ~isequal(obj.path(1, :), obj.position)
                obj.path = [obj.position; obj.path];
            end

            if size(obj.path, 1) <= 1
                obj.path = zeros(0, 2);
                obj.pathIndex = 1;
                obj.state = 'idle';
                return;
            end

            obj.pathIndex = 2;
            obj.state = 'moving';
        end

        function [reachedNextNode, currentPosition, moveInfo] = move(obj, dt, context)
            %MOVE Advance the AGV along its current path.
            % Inputs:
            %   dt      - Simulation step size in seconds.
            %   context - Optional struct with fields:
            %             map, dynamicObstacles, timeWindowManager,
            %             currentTime, waitTimeout.
            % Outputs:
            %   reachedNextNode - True if at least one path node was reached.
            %   currentPosition - Updated AGV position [row, col].
            %   moveInfo        - Struct containing avoidance/replan metadata.
            validateattributes(dt, {'numeric'}, {'scalar', 'nonnegative', 'finite'});
            if nargin < 3
                context = struct();
            end

            reachedNextNode = false;
            moveInfo = agv.AGVClass.emptyMoveInfo();

            if isempty(obj.path) || obj.pathIndex > size(obj.path, 1)
                currentPosition = obj.position;
                obj.lastMoveInfo = moveInfo;
                return;
            end

            if obj.shouldTriggerAvoidance(context)
                moveInfo = obj.handleDynamicObstacle(context);
            end

            remainingDistance = obj.speed * dt;
            while remainingDistance > 0 && obj.pathIndex <= size(obj.path, 1)
                target = obj.path(obj.pathIndex, :);
                delta = target - obj.position;
                distanceToTarget = norm(delta);

                if distanceToTarget <= eps
                    obj.position = target;
                    obj.pathIndex = obj.pathIndex + 1;
                    reachedNextNode = true;
                    continue;
                end

                if remainingDistance >= distanceToTarget
                    obj.position = target;
                    remainingDistance = remainingDistance - distanceToTarget;
                    obj.pathIndex = obj.pathIndex + 1;
                    reachedNextNode = true;
                else
                    direction = delta / distanceToTarget;
                    obj.position = obj.position + direction * remainingDistance;
                    remainingDistance = 0;
                end
            end

            if obj.pathIndex > size(obj.path, 1)
                obj.path = zeros(0, 2);
                obj.pathIndex = 1;
                obj.state = 'idle';
            elseif moveInfo.avoidanceTriggered && ~moveInfo.replannedAfterConflict
                obj.state = 'avoiding';
            elseif ~strcmp(obj.state, 'waiting')
                obj.state = 'moving';
            end

            currentPosition = obj.position;
            obj.lastMoveInfo = moveInfo;
        end

        function completed = load(obj, elapsedTime)
            %LOAD Simulate a loading operation with cumulative elapsed time.
            % Input:
            %   elapsedTime - Time slice applied to the load operation.
            % Output:
            %   completed   - True if loading finished in this call.
            completed = obj.advanceOperation('loading', 'loaded', elapsedTime, obj.loadDuration);
            if completed
                obj.isLoaded = true;
            end
        end

        function completed = unload(obj, elapsedTime)
            %UNLOAD Simulate an unloading operation with cumulative elapsed time.
            % Input:
            %   elapsedTime - Time slice applied to the unload operation.
            % Output:
            %   completed   - True if unloading finished in this call.
            completed = obj.advanceOperation('unloading', 'idle', elapsedTime, obj.unloadDuration);
            if completed
                obj.isLoaded = false;
            end
        end

        function updateState(obj, newState)
            %UPDATESTATE Update the AGV state value.
            % Input:
            %   newState - State string.
            validateattributes(newState, {'char', 'string'}, {'nonempty'});
            obj.state = char(string(newState));
            if ~any(strcmp(obj.state, {'loading', 'unloading'}))
                obj.operationRemainingTime = 0.0;
            end
        end

        function setTimeWindows(obj, timeWindows)
            %SETTIMEWINDOWS Store reserved time windows for the current path.
            % Input:
            %   timeWindows - Time window array or struct array.
            obj.timeWindows = timeWindows;
        end

        function agvStruct = toStruct(obj)
            %TOSTRUCT Serialize AGV data into a struct.
            % Output:
            %   agvStruct - Struct snapshot of the AGV.
            agvStruct = struct( ...
                'id', obj.id, ...
                'position', obj.position, ...
                'state', obj.state, ...
                'currentTask', obj.currentTask, ...
                'speed', obj.speed, ...
                'path', obj.path, ...
                'pathIndex', obj.pathIndex, ...
                'timeWindows', obj.timeWindows, ...
                'loadDuration', obj.loadDuration, ...
                'unloadDuration', obj.unloadDuration, ...
                'operationRemainingTime', obj.operationRemainingTime, ...
                'isLoaded', obj.isLoaded, ...
                'invalidatedTimeWindows', obj.invalidatedTimeWindows, ...
                'lastMoveInfo', obj.lastMoveInfo);
        end
    end

    methods (Access = private)
        function completed = advanceOperation(obj, activeState, completedState, elapsedTime, duration)
            validateattributes(elapsedTime, {'numeric'}, {'scalar', 'nonnegative', 'finite'});

            if ~strcmp(obj.state, activeState)
                obj.state = activeState;
                obj.operationRemainingTime = duration;
            end

            obj.operationRemainingTime = max(0.0, obj.operationRemainingTime - elapsedTime);
            completed = obj.operationRemainingTime <= eps;

            if completed
                obj.state = completedState;
                obj.operationRemainingTime = 0.0;
            end
        end

        function tf = shouldTriggerAvoidance(obj, context)
            tf = isstruct(context) && isfield(context, 'map') && isfield(context, 'dynamicObstacles') && ...
                ~isempty(context.dynamicObstacles) && ~isempty(obj.path) && obj.pathIndex <= size(obj.path, 1);
            if ~tf
                return;
            end

            remainingPath = obj.remainingPath();
            lookaheadCount = min(size(remainingPath, 1), obj.dynamicObstacleLookahead + 1);
            tf = false;
            for i = 2:lookaheadCount
                if agv.AGVClass.containsRow(context.dynamicObstacles, round(remainingPath(i, :)))
                    tf = true;
                    return;
                end
            end
        end

        function moveInfo = handleDynamicObstacle(obj, context)
            moveInfo = agv.AGVClass.emptyMoveInfo();
            moveInfo.avoidanceTriggered = true;
            moveInfo.strategy = 'dwa';

            currentNode = round(double(obj.position(:))');
            remainingPath = obj.remainingPath();
            taskId = obj.currentTaskId();
            [localPath, dwaInfo] = obj.dwaPlanner.planLocalPath( ...
                obj, context.map, context.dynamicObstacles, remainingPath, taskId);
            moveInfo.dwa = dwaInfo;

            if isempty(localPath)
                moveInfo.avoidanceTriggered = false;
                moveInfo.strategy = '';
                return;
            end

            currentTime = agv.AGVClass.contextValue(context, 'currentTime', 0.0);
            moveInfo.originalPath = remainingPath;
            moveInfo.localPath = localPath;

            if isfield(context, 'timeWindowManager') && ~isempty(context.timeWindowManager)
                invalidated = context.timeWindowManager.invalidatePath(obj.id, remainingPath, currentTime);
                obj.invalidatedTimeWindows = invalidated;
                moveInfo.invalidatedWindows = invalidated;
            end

            obj.replaceRemainingPath(currentNode, localPath);
            obj.state = 'avoiding';

            if isfield(context, 'timeWindowManager') && ~isempty(context.timeWindowManager)
                [reservedWindows, conflictInfo] = context.timeWindowManager.reservePath( ...
                    obj.id, obj.remainingPath(), currentTime, obj.speed);

                if isempty(conflictInfo)
                    obj.setTimeWindows(reservedWindows);
                    return;
                end

                moveInfo.conflictDetected = true;
                moveInfo.conflictInfo = conflictInfo;

                replannedPath = obj.replanAfterConflict(context, remainingPath, conflictInfo);
                if ~isempty(replannedPath)
                    obj.replaceRemainingPath(currentNode, replannedPath);
                    [replannedWindows, secondConflict] = context.timeWindowManager.reservePath( ...
                        obj.id, obj.remainingPath(), currentTime, obj.speed);
                    if isempty(secondConflict)
                        obj.setTimeWindows(replannedWindows);
                        moveInfo.replannedAfterConflict = true;
                        moveInfo.replannedPath = replannedPath;
                        moveInfo.strategy = 'replan';
                        return;
                    end
                    moveInfo.conflictInfo = secondConflict;
                end

                obj.setTimeWindows(timewindow.TimeWindowManager.emptyWindowArray());
                obj.updateState('waiting');
                return;
            end

            obj.setTimeWindows(timewindow.TimeWindowManager.emptyWindowArray());
        end

        function replannedPath = replanAfterConflict(obj, context, originalRemainingPath, conflictInfo)
            replannedPath = zeros(0, 2);
            if ~isfield(context, 'map') || isempty(context.map) || isempty(originalRemainingPath)
                return;
            end

            taskId = obj.currentTaskId();
            workingMap = pathplan.DWAClass.cloneMap(context.map);
            blockedNodes = agv.AGVClass.uniqueRows([ ...
                double(context.dynamicObstacles); ...
                double(conflictInfo.existingWindow.edgeIndex(1:2)); ...
                double(conflictInfo.existingWindow.edgeIndex(3:4))]);
            pathplan.DWAClass.applyObstacleNodes(workingMap, blockedNodes, round(obj.position));

            currentNode = round(double(obj.position(:))');
            goalNode = originalRemainingPath(end, :);
            replannedPath = pathplan.AStar(workingMap, currentNode, goalNode, obj.id, taskId);
        end

        function remainingPath = remainingPath(obj)
            currentNode = round(double(obj.position(:))');
            if isempty(obj.path) || obj.pathIndex > size(obj.path, 1)
                remainingPath = currentNode;
                return;
            end

            remainingPath = obj.path(max(1, obj.pathIndex - 1):end, :);
            if ~isequal(remainingPath(1, :), currentNode)
                remainingPath = [currentNode; remainingPath];
            end
            remainingPath = agv.AGVClass.removeSequentialDuplicates(remainingPath);
        end

        function replaceRemainingPath(obj, currentNode, newRemainingPath)
            prefixEnd = max(1, obj.pathIndex - 1);
            if isempty(obj.path)
                prefix = currentNode;
            else
                prefix = obj.path(1:prefixEnd, :);
                if ~isequal(prefix(end, :), currentNode)
                    prefix(end, :) = currentNode;
                end
            end

            suffix = double(newRemainingPath);
            if isempty(suffix)
                suffix = currentNode;
            end
            if ~isequal(suffix(1, :), currentNode)
                suffix = [currentNode; suffix];
            end

            obj.path = agv.AGVClass.removeSequentialDuplicates([prefix; suffix(2:end, :)]);
            obj.pathIndex = size(prefix, 1) + 1;
            if obj.pathIndex > size(obj.path, 1)
                obj.pathIndex = size(obj.path, 1);
            end
        end

        function taskId = currentTaskId(obj)
            taskId = [];
            if isa(obj.currentTask, 'task.TaskClass') && ~isempty(obj.currentTask)
                taskId = obj.currentTask(1).id;
            end
        end
    end

    methods (Static)
        function agvPool = createDefaultPool()
            %CREATEDEFAULTPOOL Create the default AGV pool configuration.
            positions = map.MapClass.defaultAGVPositions();
            agvPool = repmat(agv.AGVClass(1, positions(1, :), 1.0), 0, 1);
            for i = 1:size(positions, 1)
                agvPool(i, 1) = agv.AGVClass(i, positions(i, :), 1.0); %#ok<AGROW>
            end
        end

        function agvPoolData = poolToStructArray(agvPool)
            %POOLTOSTRUCTARRAY Convert an AGV object array to a struct array.
            if isempty(agvPool)
                agvPoolData = repmat(struct( ...
                    'id', [], ...
                    'position', [], ...
                    'state', '', ...
                    'currentTask', [], ...
                    'speed', [], ...
                    'path', [], ...
                    'pathIndex', [], ...
                    'timeWindows', [], ...
                    'loadDuration', [], ...
                    'unloadDuration', [], ...
                    'operationRemainingTime', [], ...
                    'isLoaded', [], ...
                    'invalidatedTimeWindows', [], ...
                    'lastMoveInfo', []), 0, 1);
                return;
            end

            firstEntry = agvPool(1).toStruct();
            agvPoolData = repmat(firstEntry, numel(agvPool), 1);
            for i = 2:numel(agvPool)
                agvPoolData(i, 1) = agvPool(i).toStruct();
            end
        end

        function moveInfo = emptyMoveInfo()
            %EMPTYMOVEINFO Return a standardized move metadata struct.
            moveInfo = struct( ...
                'avoidanceTriggered', false, ...
                'strategy', '', ...
                'originalPath', zeros(0, 2), ...
                'localPath', zeros(0, 2), ...
                'replannedPath', zeros(0, 2), ...
                'invalidatedWindows', timewindow.TimeWindowManager.emptyWindowArray(), ...
                'conflictDetected', false, ...
                'conflictInfo', [], ...
                'replannedAfterConflict', false, ...
                'dwa', struct());
        end

        function value = contextValue(context, fieldName, defaultValue)
            %CONTEXTVALUE Read a context field or return a default.
            if isstruct(context) && isfield(context, fieldName) && ~isempty(context.(fieldName))
                value = context.(fieldName);
            else
                value = defaultValue;
            end
        end

        function tf = containsRow(matrix, rowValue)
            %CONTAINSROW Return true when a row exists in a matrix.
            tf = ~isempty(matrix) && any(all(double(matrix) == double(rowValue), 2));
        end

        function matrix = uniqueRows(matrix)
            %UNIQUEROWS Return unique rows while preserving order.
            if isempty(matrix)
                return;
            end

            [~, uniqueIdx] = unique(double(matrix), 'rows', 'stable');
            matrix = double(matrix(sort(uniqueIdx), :));
        end

        function path = removeSequentialDuplicates(path)
            %REMOVESEQUENTIALDUPLICATES Remove consecutive repeated nodes.
            if size(path, 1) <= 1
                return;
            end

            keepMask = true(size(path, 1), 1);
            for i = 2:size(path, 1)
                keepMask(i) = ~isequal(path(i, :), path(i - 1, :));
            end
            path = path(keepMask, :);
        end
    end
end
