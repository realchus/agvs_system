classdef TaskClass < handle
    %TASKCLASS Task model for AGV dispatch and execution.
    %   A task contains a start position, an ordered waypoint sequence,
    %   base priority, request time, and runtime status.

    properties
        id
        start
        waypoints
        priority
        requestTime
        status
    end

    methods
        function obj = TaskClass(id, startPos, waypoints, priority, requestTime, status)
            %TASKCLASS Construct a task object.
            % Inputs:
            %   id          - Task identifier.
            %   startPos    - 1-by-2 task start coordinate [row, col].
            %   waypoints   - Struct array with fields name and position, or
            %                 an N-by-2 numeric waypoint matrix.
            %   priority    - Numeric base priority.
            %   requestTime - Task request time in seconds.
            %   status      - Task runtime status string.
            if nargin < 1
                error('TaskClass:MissingId', 'Task id is required.');
            end
            if nargin < 2 || isempty(startPos)
                startPos = [1, 1];
            end
            if nargin < 3 || isempty(waypoints)
                waypoints = struct('name', {}, 'position', {});
            end
            if nargin < 4 || isempty(priority)
                priority = 1;
            end
            if nargin < 5 || isempty(requestTime)
                requestTime = 0;
            end
            if nargin < 6 || isempty(status)
                status = 'pending';
            end

            validateattributes(id, {'numeric'}, {'scalar', 'integer', 'positive'});
            validateattributes(startPos, {'numeric'}, {'vector', 'numel', 2, 'finite', 'positive'});
            validateattributes(priority, {'numeric'}, {'scalar', 'finite', 'nonnegative'});
            validateattributes(requestTime, {'numeric'}, {'scalar', 'finite', 'nonnegative'});
            validateattributes(status, {'char', 'string'}, {'nonempty'});

            obj.id = double(id);
            obj.start = double(startPos(:))';
            obj.waypoints = obj.normalizeWaypoints(waypoints);
            obj.priority = double(priority);
            obj.requestTime = double(requestTime);
            obj.status = char(string(status));
        end

        function updateStatus(obj, newStatus)
            %UPDATESTATUS Update the task status string.
            % Input:
            %   newStatus - New task state, such as pending or completed.
            validateattributes(newStatus, {'char', 'string'}, {'nonempty'});
            obj.status = char(string(newStatus));
        end

        function positions = getWaypointPositions(obj)
            %GETWAYPOINTPOSITIONS Return the ordered waypoint coordinates.
            % Output:
            %   positions - N-by-2 waypoint position matrix.
            if isempty(obj.waypoints)
                positions = zeros(0, 2);
                return;
            end

            positions = reshape([obj.waypoints.position], 2, []).';
        end

        function names = getWaypointNames(obj)
            %GETWAYPOINTNAMES Return the ordered waypoint names.
            % Output:
            %   names - Cell array of waypoint names.
            names = {obj.waypoints.name}.';
        end

        function taskStruct = toStruct(obj)
            %TOSTRUCT Serialize task data into a struct.
            % Output:
            %   taskStruct - Struct snapshot of the task.
            taskStruct = struct( ...
                'id', obj.id, ...
                'start', obj.start, ...
                'waypoints', obj.waypoints, ...
                'priority', obj.priority, ...
                'requestTime', obj.requestTime, ...
                'status', obj.status);
        end
    end

    methods (Access = private)
        function waypointsOut = normalizeWaypoints(~, waypointsIn)
            if isempty(waypointsIn)
                waypointsOut = struct('name', {}, 'position', {});
                return;
            end

            if isnumeric(waypointsIn)
                validateattributes(waypointsIn, {'numeric'}, {'2d', 'ncols', 2, 'finite', 'positive'});
                waypointsOut = repmat(struct('name', '', 'position', [0, 0]), size(waypointsIn, 1), 1);
                for i = 1:size(waypointsIn, 1)
                    waypointsOut(i, 1) = struct( ...
                        'name', sprintf('Waypoint%d', i), ...
                        'position', double(waypointsIn(i, :)));
                end
                return;
            end

            if ~isstruct(waypointsIn) || ~all(isfield(waypointsIn, {'name', 'position'}))
                error('TaskClass:InvalidWaypoints', ...
                    'Waypoints must be an N-by-2 matrix or a struct array with fields name and position.');
            end

            waypointsOut = repmat(struct('name', '', 'position', [0, 0]), numel(waypointsIn), 1);
            for i = 1:numel(waypointsIn)
                position = double(waypointsIn(i).position(:))';
                validateattributes(position, {'numeric'}, {'vector', 'numel', 2, 'finite', 'positive'});
                waypointName = char(string(waypointsIn(i).name));
                if isempty(waypointName)
                    waypointName = sprintf('Waypoint%d', i);
                end

                waypointsOut(i, 1) = struct('name', waypointName, 'position', position);
            end
        end
    end

    methods (Static)
        function tasksData = toStructArray(tasks)
            %TOSTRUCTARRAY Convert a task object array to a struct array.
            if isempty(tasks)
                tasksData = repmat(struct( ...
                    'id', [], ...
                    'start', [], ...
                    'waypoints', struct('name', {}, 'position', {}), ...
                    'priority', [], ...
                    'requestTime', [], ...
                    'status', ''), 0, 1);
                return;
            end

            firstEntry = tasks(1).toStruct();
            tasksData = repmat(firstEntry, numel(tasks), 1);
            for i = 2:numel(tasks)
                tasksData(i, 1) = tasks(i).toStruct();
            end
        end
    end
end
