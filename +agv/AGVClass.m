classdef AGVClass < handle
    %AGVCLASS AGV model with movement, task, and load/unload state handling.
    %   The class stores AGV runtime status and can advance along a grid
    %   path using a fixed speed model.

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
                if strcmp(obj.state, 'moving')
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

        function [reachedNextNode, currentPosition] = move(obj, dt)
            %MOVE Advance the AGV along its current path.
            % Input:
            %   dt - Simulation step size in seconds.
            % Outputs:
            %   reachedNextNode - True if at least one path node was reached.
            %   currentPosition - Updated AGV position [row, col].
            validateattributes(dt, {'numeric'}, {'scalar', 'nonnegative', 'finite'});

            reachedNextNode = false;

            if isempty(obj.path) || obj.pathIndex > size(obj.path, 1)
                currentPosition = obj.position;
                return;
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
            end

            currentPosition = obj.position;
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
                'isLoaded', obj.isLoaded);
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
                    'isLoaded', []), 0, 1);
                return;
            end

            firstEntry = agvPool(1).toStruct();
            agvPoolData = repmat(firstEntry, numel(agvPool), 1);
            for i = 2:numel(agvPool)
                agvPoolData(i, 1) = agvPool(i).toStruct();
            end
        end
    end
end
