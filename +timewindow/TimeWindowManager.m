classdef TimeWindowManager < handle
    %TIMEWINDOWMANAGER Manage path reservation windows for AGVs.
    %   The manager stores edge occupancy windows and can detect conflicts
    %   caused by overlapping use of the same path segment.

    properties
        timeWindows
    end

    methods
        function obj = TimeWindowManager()
            %TIMEWINDOWMANAGER Construct an empty time window manager.
            obj.timeWindows = timewindow.TimeWindowManager.emptyWindowArray();
        end

        function window = addTimeWindow(obj, edgeIndex, startTime, endTime, agvId, direction)
            %ADDTIMEWINDOW Add a single time window entry to the manager.
            % Inputs:
            %   edgeIndex - 1-by-4 edge definition [r1 c1 r2 c2].
            %   startTime - Reservation start time.
            %   endTime   - Reservation end time.
            %   agvId     - AGV identifier.
            %   direction - Segment direction code.
            % Output:
            %   window    - Struct representation of the added time window.
            window = timewindow.TimeWindowManager.buildWindow( ...
                edgeIndex, startTime, endTime, agvId, direction);

            obj.timeWindows(end + 1, 1) = window; %#ok<AGROW>
        end

        function [hasConflict, conflictInfo] = detectConflict(obj, newWindow, existingWindows)
            %DETECTCONFLICT Detect overlap conflicts for a candidate window.
            % Inputs:
            %   newWindow      - Candidate time window struct.
            %   existingWindows - Optional existing window array.
            % Outputs:
            %   hasConflict    - True if a conflict was found.
            %   conflictInfo   - Struct describing the first conflict found.
            if nargin < 3 || isempty(existingWindows)
                existingWindows = obj.timeWindows;
            end

            newWindow = timewindow.TimeWindowManager.normalizeWindow(newWindow);
            conflictInfo = struct( ...
                'type', '', ...
                'message', '', ...
                'newWindow', newWindow, ...
                'existingWindow', timewindow.TimeWindowManager.emptyWindowArray());

            hasConflict = false;
            for i = 1:numel(existingWindows)
                existing = timewindow.TimeWindowManager.normalizeWindow(existingWindows(i));

                if existing.agvId == newWindow.agvId
                    continue;
                end

                if ~timewindow.TimeWindowManager.isSameUndirectedEdge(existing.edgeIndex, newWindow.edgeIndex)
                    continue;
                end

                if ~timewindow.TimeWindowManager.hasTimeOverlap(existing.startTime, existing.endTime, ...
                        newWindow.startTime, newWindow.endTime)
                    continue;
                end

                hasConflict = true;
                if existing.direction == newWindow.direction
                    conflictType = 'same_direction_overlap';
                else
                    conflictType = 'opposite_direction_overlap';
                end

                conflictInfo.type = conflictType;
                conflictInfo.message = sprintf( ...
                    'Conflict on edge [%d %d %d %d] between AGV %d and AGV %d.', ...
                    newWindow.edgeIndex(1), newWindow.edgeIndex(2), ...
                    newWindow.edgeIndex(3), newWindow.edgeIndex(4), ...
                    newWindow.agvId, existing.agvId);
                conflictInfo.existingWindow = existing;
                return;
            end
        end

        function [reservedWindows, conflictInfo] = reservePath(obj, agvId, path, startTime, speed)
            %RESERVEPATH Reserve time windows for every edge in a path.
            % Inputs:
            %   agvId     - AGV identifier.
            %   path      - N-by-2 path coordinates.
            %   startTime - Path start time.
            %   speed     - Traversal speed in grid units per second.
            % Outputs:
            %   reservedWindows - Reserved window array. Empty on conflict.
            %   conflictInfo    - Empty if reservation succeeds.
            validateattributes(path, {'numeric'}, {'2d', 'ncols', 2, 'finite'});
            validateattributes(startTime, {'numeric'}, {'scalar', 'finite', 'nonnegative'});
            validateattributes(speed, {'numeric'}, {'scalar', 'positive', 'finite'});

            if size(path, 1) < 2
                reservedWindows = timewindow.TimeWindowManager.emptyWindowArray();
                conflictInfo = [];
                return;
            end

            candidateWindows = timewindow.TimeWindowManager.emptyWindowArray();
            currentTime = double(startTime);

            for i = 1:(size(path, 1) - 1)
                startNode = double(path(i, :));
                endNode = double(path(i + 1, :));
                segmentLength = norm(endNode - startNode);
                if segmentLength <= eps
                    continue;
                end

                edgeIndex = [startNode, endNode];
                direction = timewindow.TimeWindowManager.computeDirection(startNode, endNode);
                endTime = currentTime + segmentLength / speed;
                candidate = timewindow.TimeWindowManager.buildWindow( ...
                    edgeIndex, currentTime, endTime, agvId, direction);

                [hasConflict, conflictInfo] = obj.detectConflict(candidate, [obj.timeWindows; candidateWindows]); %#ok<AGROW>
                if hasConflict
                    reservedWindows = timewindow.TimeWindowManager.emptyWindowArray();
                    return;
                end

                candidateWindows(end + 1, 1) = candidate; %#ok<AGROW>
                currentTime = endTime;
            end

            if isempty(candidateWindows)
                reservedWindows = timewindow.TimeWindowManager.emptyWindowArray();
                conflictInfo = [];
                return;
            end

            obj.timeWindows = [obj.timeWindows; candidateWindows];
            reservedWindows = candidateWindows;
            conflictInfo = [];
        end

        function releasePath(obj, agvId, path)
            %RELEASEPATH Release reserved windows for an AGV path.
            % Inputs:
            %   agvId - AGV identifier.
            %   path  - Optional N-by-2 path. If omitted, remove all windows
            %           belonging to the AGV.
            if isempty(obj.timeWindows)
                return;
            end

            keepMask = true(numel(obj.timeWindows), 1);

            if nargin < 3 || isempty(path)
                for i = 1:numel(obj.timeWindows)
                    if obj.timeWindows(i).agvId == agvId
                        keepMask(i) = false;
                    end
                end
                obj.timeWindows = obj.timeWindows(keepMask);
                return;
            end

            validateattributes(path, {'numeric'}, {'2d', 'ncols', 2, 'finite'});
            releaseEdges = timewindow.TimeWindowManager.pathToEdgeIndexList(path);

            for i = 1:numel(obj.timeWindows)
                if obj.timeWindows(i).agvId ~= agvId
                    continue;
                end

                for j = 1:size(releaseEdges, 1)
                    if timewindow.TimeWindowManager.isSameDirectedEdge( ...
                            obj.timeWindows(i).edgeIndex, releaseEdges(j, :))
                        keepMask(i) = false;
                        break;
                    end
                end
            end

            obj.timeWindows = obj.timeWindows(keepMask);
        end
    end

    methods (Static)
        function windows = emptyWindowArray()
            %EMPTYWINDOWARRAY Return an empty window struct array.
            windows = repmat(struct( ...
                'edgeIndex', zeros(1, 4), ...
                'startTime', 0, ...
                'endTime', 0, ...
                'agvId', 0, ...
                'direction', 0), 0, 1);
        end

        function direction = computeDirection(startNode, endNode)
            %COMPUTEDIRECTION Convert a grid segment into a direction code.
            % Output mapping: 0=east, 1=south, 2=west, 3=north.
            delta = double(endNode) - double(startNode);
            if isequal(delta, [0, 1])
                direction = 0;
            elseif isequal(delta, [1, 0])
                direction = 1;
            elseif isequal(delta, [0, -1])
                direction = 2;
            elseif isequal(delta, [-1, 0])
                direction = 3;
            else
                error('TimeWindowManager:InvalidEdge', ...
                    'Time windows require axis-aligned unit segments.');
            end
        end

        function edgeList = pathToEdgeIndexList(path)
            %PATHTOEDGEINDEXLIST Convert a path to directed edge rows.
            if size(path, 1) < 2
                edgeList = zeros(0, 4);
                return;
            end

            edgeList = zeros(size(path, 1) - 1, 4);
            for i = 1:(size(path, 1) - 1)
                edgeList(i, :) = [path(i, :), path(i + 1, :)];
            end
        end

        function window = buildWindow(edgeIndex, startTime, endTime, agvId, direction)
            %BUILDWINDOW Create and validate a time window struct.
            validateattributes(edgeIndex, {'numeric'}, {'vector', 'numel', 4, 'finite'});
            validateattributes(startTime, {'numeric'}, {'scalar', 'finite', 'nonnegative'});
            validateattributes(endTime, {'numeric'}, {'scalar', 'finite', '>', startTime});
            validateattributes(agvId, {'numeric'}, {'scalar', 'integer', 'positive'});
            validateattributes(direction, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 3});

            window = struct( ...
                'edgeIndex', double(edgeIndex(:))', ...
                'startTime', double(startTime), ...
                'endTime', double(endTime), ...
                'agvId', double(agvId), ...
                'direction', double(direction));
        end

        function tf = hasTimeOverlap(startA, endA, startB, endB)
            %HASTIMEOVERLAP Return true when two time intervals overlap.
            tf = startA < endB && startB < endA;
        end

        function tf = isSameDirectedEdge(edgeA, edgeB)
            %ISSAMEDIRECTEDEDGE Compare two directed edges.
            tf = isequal(double(edgeA(:))', double(edgeB(:))');
        end

        function tf = isSameUndirectedEdge(edgeA, edgeB)
            %ISSAMEUNDIRECTEDEDGE Compare two edges ignoring direction.
            edgeA = double(edgeA(:))';
            edgeB = double(edgeB(:))';
            tf = isequal(edgeA, edgeB) || isequal(edgeA, [edgeB(3:4), edgeB(1:2)]);
        end

        function window = normalizeWindow(window)
            %NORMALIZEWINDOW Validate and normalize a time window struct.
            requiredFields = {'edgeIndex', 'startTime', 'endTime', 'agvId', 'direction'};
            if ~isstruct(window) || ~all(isfield(window, requiredFields))
                error('TimeWindowManager:InvalidWindow', ...
                    'Time window structs must define edgeIndex, startTime, endTime, agvId, and direction.');
            end

            window = timewindow.TimeWindowManager.buildWindow( ...
                window.edgeIndex, window.startTime, window.endTime, window.agvId, window.direction);
        end
    end
end
