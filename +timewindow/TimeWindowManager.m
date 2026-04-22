classdef TimeWindowManager < handle
    %TIMEWINDOWMANAGER Manage path reservation windows for AGVs.
    %   The manager stores edge occupancy windows and can detect conflicts
    %   caused by overlapping use of the same path segment.

    properties
        timeWindows
        invalidatedWindows
        timeWindowIndex
        invalidatedWindowIndex
    end

    methods
        function obj = TimeWindowManager()
            %TIMEWINDOWMANAGER Construct an empty time window manager.
            obj.timeWindows = timewindow.TimeWindowManager.emptyWindowArray();
            obj.invalidatedWindows = timewindow.TimeWindowManager.emptyWindowArray();
            obj.timeWindowIndex = timewindow.TimeWindowManager.emptyWindowIndex();
            obj.invalidatedWindowIndex = timewindow.TimeWindowManager.emptyWindowIndex();
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

            obj.addReservedWindow(window);
        end

        function window = addNodeTimeWindow(obj, nodeIndex, startTime, endTime, agvId)
            %ADDNODETIMEWINDOW Add a single node occupancy window.
            % Inputs:
            %   nodeIndex - 1-by-2 node coordinate [row col].
            %   startTime - Reservation start time.
            %   endTime   - Reservation end time.
            %   agvId     - AGV identifier.
            % Output:
            %   window    - Struct representation of the added node window.
            window = timewindow.TimeWindowManager.buildNodeWindow( ...
                nodeIndex, startTime, endTime, agvId);

            obj.addReservedWindow(window);
        end

        function addReservedWindow(obj, windows)
            %ADDRESERVEDWINDOW Append active windows and update the index.
            if isempty(windows)
                return;
            end

            windows = timewindow.TimeWindowManager.normalizeWindowArray(windows);
            obj.timeWindows = [obj.timeWindows; windows];
            obj.timeWindowIndex = timewindow.TimeWindowManager.insertWindowsIntoIndex( ...
                obj.timeWindowIndex, windows);
        end

        function addInvalidatedWindow(obj, windows)
            %ADDINVALIDATEDWINDOW Append blocked windows and update the index.
            if isempty(windows)
                return;
            end

            windows = timewindow.TimeWindowManager.normalizeWindowArray(windows);
            obj.invalidatedWindows = [obj.invalidatedWindows; windows];
            obj.invalidatedWindowIndex = timewindow.TimeWindowManager.insertWindowsIntoIndex( ...
                obj.invalidatedWindowIndex, windows);
        end

        function removeReservedWindow(obj, targetWindow)
            %REMOVERESERVEDWINDOW Remove matching active windows and refresh the index.
            if isempty(obj.timeWindows) || isempty(targetWindow)
                return;
            end

            keepMask = true(numel(obj.timeWindows), 1);
            for i = 1:numel(obj.timeWindows)
                for j = 1:numel(targetWindow)
                    if timewindow.TimeWindowManager.isSameWindow(obj.timeWindows(i), targetWindow(j))
                        keepMask(i) = false;
                        break;
                    end
                end
            end
            obj.timeWindows = obj.timeWindows(keepMask);
            obj.refreshIndexes();
        end

        function refreshIndexes(obj)
            %REFRESHINDEXES Rebuild persistent indexes after bulk mutation.
            obj.timeWindows = timewindow.TimeWindowManager.normalizeWindowArray(obj.timeWindows);
            obj.invalidatedWindows = timewindow.TimeWindowManager.normalizeWindowArray(obj.invalidatedWindows);
            obj.timeWindowIndex = timewindow.TimeWindowManager.buildWindowIndex(obj.timeWindows);
            obj.invalidatedWindowIndex = timewindow.TimeWindowManager.buildWindowIndex(obj.invalidatedWindows);
        end

        function [hasConflict, conflictInfo] = detectConflict(obj, newWindow, existingWindows, existingIndex)
            %DETECTCONFLICT Detect overlap conflicts for a candidate window.
            % Inputs:
            %   newWindow       - Candidate time window struct.
            %   existingWindows - Optional existing window array.
            %   existingIndex   - Optional prebuilt edge index for windows.
            % Outputs:
            %   hasConflict     - True if a conflict was found.
            %   conflictInfo    - Struct describing the first conflict found.
            if nargin < 3
                existingWindows = [];
            end
            hasExistingIndex = nargin >= 4 && isa(existingIndex, 'containers.Map');
            if nargin < 4
                existingIndex = [];
            end

            newWindow = timewindow.TimeWindowManager.normalizeWindow(newWindow);
            conflictInfo = struct( ...
                'type', '', ...
                'message', '', ...
                'newWindow', newWindow, ...
                'existingWindow', timewindow.TimeWindowManager.emptyWindowArray());

            if ~hasExistingIndex && ~isempty(existingWindows)
                existingIndex = timewindow.TimeWindowManager.buildWindowIndex(existingWindows);
            end

            if hasExistingIndex || ~isempty(existingWindows)
                [hasConflict, existing] = timewindow.TimeWindowManager.findConflictInIndex(newWindow, existingIndex);
            else
                [hasConflict, existing] = timewindow.TimeWindowManager.findConflictInIndex(newWindow, obj.timeWindowIndex);
                if ~hasConflict
                    [hasConflict, existing] = timewindow.TimeWindowManager.findConflictInIndex(newWindow, obj.invalidatedWindowIndex);
                end
            end
            if ~hasConflict
                return;
            end

            if strcmp(newWindow.windowType, 'node')
                if existing.agvId == 0
                    conflictType = 'invalidated_node';
                    message = sprintf( ...
                        'Blocked node [%d %d] overlaps AGV %d reservation.', ...
                        newWindow.nodeIndex(1), newWindow.nodeIndex(2), newWindow.agvId);
                else
                    conflictType = 'node_overlap';
                    message = sprintf( ...
                        'Conflict on node [%d %d] between AGV %d and AGV %d.', ...
                        newWindow.nodeIndex(1), newWindow.nodeIndex(2), ...
                        newWindow.agvId, existing.agvId);
                end
            else
                if existing.agvId == 0
                    conflictType = 'invalidated_edge';
                    message = sprintf( ...
                        'Blocked edge [%d %d %d %d] overlaps AGV %d reservation.', ...
                        newWindow.edgeIndex(1), newWindow.edgeIndex(2), ...
                        newWindow.edgeIndex(3), newWindow.edgeIndex(4), ...
                        newWindow.agvId);
                elseif existing.direction == newWindow.direction
                    conflictType = 'same_direction_overlap';
                    message = sprintf( ...
                        'Conflict on edge [%d %d %d %d] between AGV %d and AGV %d.', ...
                        newWindow.edgeIndex(1), newWindow.edgeIndex(2), ...
                        newWindow.edgeIndex(3), newWindow.edgeIndex(4), ...
                        newWindow.agvId, existing.agvId);
                else
                    conflictType = 'opposite_direction_overlap';
                    message = sprintf( ...
                        'Conflict on edge [%d %d %d %d] between AGV %d and AGV %d.', ...
                        newWindow.edgeIndex(1), newWindow.edgeIndex(2), ...
                        newWindow.edgeIndex(3), newWindow.edgeIndex(4), ...
                        newWindow.agvId, existing.agvId);
                end
            end

            conflictInfo.type = conflictType;
            conflictInfo.message = message;
            conflictInfo.existingWindow = existing;
        end

        function [reservedWindows, conflictInfo] = reservePath(obj, agvId, path, startTime, speed, nodeDwellTimes)
            %RESERVEPATH Reserve time windows for every edge in a path.
            % Inputs:
            %   agvId          - AGV identifier.
            %   path           - N-by-2 path coordinates.
            %   startTime      - Path start time.
            %   speed          - Traversal speed in grid units per second.
            %   nodeDwellTimes - Optional N-by-1 dwell seconds at path nodes.
            % Outputs:
            %   reservedWindows - Reserved window array. Empty on conflict.
            %   conflictInfo    - Empty if reservation succeeds.
            validateattributes(path, {'numeric'}, {'2d', 'ncols', 2, 'finite'});
            validateattributes(startTime, {'numeric'}, {'scalar', 'finite', 'nonnegative'});
            validateattributes(speed, {'numeric'}, {'scalar', 'positive', 'finite'});
            if nargin < 6 || isempty(nodeDwellTimes)
                nodeDwellTimes = zeros(size(path, 1), 1);
            end
            validateattributes(nodeDwellTimes, {'numeric'}, ...
                {'vector', 'numel', size(path, 1), 'finite', 'nonnegative'});
            nodeDwellTimes = double(nodeDwellTimes(:));

            if size(path, 1) < 2
                reservedWindows = timewindow.TimeWindowManager.emptyWindowArray();
                conflictInfo = [];
                return;
            end

            activeIndex = obj.timeWindowIndex;
            invalidatedIndex = obj.invalidatedWindowIndex;
            candidateIndex = timewindow.TimeWindowManager.emptyWindowIndex();
            candidateWindows = timewindow.TimeWindowManager.emptyWindowArray();
            currentTime = double(startTime);

            for i = 1:(size(path, 1) - 1)
                startNode = double(path(i, :));
                endNode = double(path(i + 1, :));

                dwellEndTime = currentTime + nodeDwellTimes(i);
                if dwellEndTime > currentTime
                    candidate = timewindow.TimeWindowManager.buildNodeWindow( ...
                        startNode, currentTime, dwellEndTime, agvId);
                    [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], activeIndex);
                    if ~hasConflict
                        [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], invalidatedIndex);
                    end
                    if ~hasConflict
                        [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], candidateIndex);
                    end
                    if hasConflict
                        reservedWindows = timewindow.TimeWindowManager.emptyWindowArray();
                        return;
                    end

                    candidateWindows(end + 1, 1) = candidate; %#ok<AGROW>
                    candidateIndex = timewindow.TimeWindowManager.insertWindowIntoIndex(candidateIndex, candidate);
                    currentTime = dwellEndTime;
                end

                segmentLength = norm(endNode - startNode);
                if segmentLength <= eps
                    continue;
                end

                edgeIndex = [startNode, endNode];
                direction = timewindow.TimeWindowManager.computeDirection(startNode, endNode);
                endTime = currentTime + segmentLength / speed;
                segmentWindows = [ ...
                    timewindow.TimeWindowManager.buildNodeWindow(startNode, currentTime, endTime, agvId); ...
                    timewindow.TimeWindowManager.buildWindow(edgeIndex, currentTime, endTime, agvId, direction); ...
                    timewindow.TimeWindowManager.buildNodeWindow(endNode, currentTime, endTime, agvId)];

                for j = 1:numel(segmentWindows)
                    candidate = segmentWindows(j);
                    [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], activeIndex);
                    if ~hasConflict
                        [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], invalidatedIndex);
                    end
                    if ~hasConflict
                        [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], candidateIndex);
                    end
                    if hasConflict
                        reservedWindows = timewindow.TimeWindowManager.emptyWindowArray();
                        return;
                    end

                    candidateWindows(end + 1, 1) = candidate; %#ok<AGROW>
                    candidateIndex = timewindow.TimeWindowManager.insertWindowIntoIndex(candidateIndex, candidate);
                end
                currentTime = endTime;
            end

            finalDwellEndTime = currentTime + nodeDwellTimes(end);
            if finalDwellEndTime > currentTime
                candidate = timewindow.TimeWindowManager.buildNodeWindow( ...
                    double(path(end, :)), currentTime, finalDwellEndTime, agvId);
                [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], activeIndex);
                if ~hasConflict
                    [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], invalidatedIndex);
                end
                if ~hasConflict
                    [hasConflict, conflictInfo] = obj.detectConflict(candidate, [], candidateIndex);
                end
                if hasConflict
                    reservedWindows = timewindow.TimeWindowManager.emptyWindowArray();
                    return;
                end

                candidateWindows(end + 1, 1) = candidate; %#ok<AGROW>
            end

            if isempty(candidateWindows)
                reservedWindows = timewindow.TimeWindowManager.emptyWindowArray();
                conflictInfo = [];
                return;
            end

            obj.addReservedWindow(candidateWindows);
            reservedWindows = candidateWindows;
            conflictInfo = [];
        end

        function invalidated = invalidatePath(obj, agvId, path, blockStartTime)
            %INVALIDATEPATH Mark future reserved windows as unavailable.
            % Inputs:
            %   agvId          - AGV identifier owning the original path.
            %   path           - Path whose remaining windows should be blocked.
            %   blockStartTime - Start time from which the windows become invalid.
            % Output:
            %   invalidated    - Window array moved to the invalid list.
            if nargin < 4 || isempty(blockStartTime)
                blockStartTime = 0.0;
            end

            invalidated = timewindow.TimeWindowManager.emptyWindowArray();
            if isempty(obj.timeWindows) || isempty(path)
                return;
            end

            releaseEdges = timewindow.TimeWindowManager.pathToEdgeIndexList(path);
            releaseKeySet = timewindow.TimeWindowManager.edgeKeySet(releaseEdges, false);
            releaseNodeSet = timewindow.TimeWindowManager.nodeKeySet(path);
            currentNode = double(path(1, :));
            keepMask = true(numel(obj.timeWindows), 1);

            for i = 1:numel(obj.timeWindows)
                existing = obj.timeWindows(i);
                if existing.agvId ~= agvId || existing.endTime <= blockStartTime
                    continue;
                end
                if strcmp(existing.windowType, 'node') && isequal(existing.nodeIndex, currentNode) && ...
                        existing.startTime <= blockStartTime && blockStartTime < existing.endTime
                    continue;
                end
                if ~timewindow.TimeWindowManager.windowMatchesPath(existing, releaseKeySet, releaseNodeSet)
                    continue;
                end

                keepMask(i) = false;
                invalidStart = max(existing.startTime, blockStartTime);
                if invalidStart < existing.endTime
                    blockedWindow = timewindow.TimeWindowManager.cloneWindowWithTiming( ...
                        existing, invalidStart, existing.endTime, 0);
                    invalidated(end + 1, 1) = blockedWindow; %#ok<AGROW>
                end
            end

            obj.timeWindows = obj.timeWindows(keepMask);
            obj.timeWindowIndex = timewindow.TimeWindowManager.buildWindowIndex(obj.timeWindows);
            if ~isempty(invalidated)
                obj.addInvalidatedWindow(invalidated);
            else
                obj.invalidatedWindowIndex = timewindow.TimeWindowManager.buildWindowIndex(obj.invalidatedWindows);
            end
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
                obj.refreshIndexes();
                return;
            end

            validateattributes(path, {'numeric'}, {'2d', 'ncols', 2, 'finite'});
            releaseEdges = timewindow.TimeWindowManager.pathToEdgeIndexList(path);
            releaseKeySet = timewindow.TimeWindowManager.edgeKeySet(releaseEdges, false);
            releaseNodeSet = timewindow.TimeWindowManager.nodeKeySet(path);

            for i = 1:numel(obj.timeWindows)
                if obj.timeWindows(i).agvId ~= agvId
                    continue;
                end
                if timewindow.TimeWindowManager.windowMatchesPath(obj.timeWindows(i), releaseKeySet, releaseNodeSet)
                    keepMask(i) = false;
                end
            end

            obj.timeWindows = obj.timeWindows(keepMask);
            obj.refreshIndexes();
        end

        function windows = getAllWindows(obj)
            %GETALLWINDOWS Return both reserved and invalidated windows.
            windows = [obj.timeWindows; obj.invalidatedWindows];
        end
    end

    methods (Static)
        function windows = emptyWindowArray()
            %EMPTYWINDOWARRAY Return an empty window struct array.
            windows = repmat(struct( ...
                'edgeIndex', zeros(1, 4), ...
                'nodeIndex', zeros(1, 2), ...
                'startTime', 0, ...
                'endTime', 0, ...
                'agvId', 0, ...
                'direction', 0, ...
                'windowType', 'edge'), 0, 1);
        end

        function indexMap = emptyWindowIndex()
            %EMPTYWINDOWINDEX Return an empty edge-to-window index map.
            indexMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
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
            validateattributes(agvId, {'numeric'}, {'scalar', 'integer', '>=', 0});
            validateattributes(direction, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 3});

            window = struct( ...
                'edgeIndex', double(edgeIndex(:))', ...
                'nodeIndex', zeros(1, 2), ...
                'startTime', double(startTime), ...
                'endTime', double(endTime), ...
                'agvId', double(agvId), ...
                'direction', double(direction), ...
                'windowType', 'edge');
        end

        function window = buildNodeWindow(nodeIndex, startTime, endTime, agvId)
            %BUILDNODEWINDOW Create and validate a node occupancy window.
            validateattributes(nodeIndex, {'numeric'}, {'vector', 'numel', 2, 'finite'});
            validateattributes(startTime, {'numeric'}, {'scalar', 'finite', 'nonnegative'});
            validateattributes(endTime, {'numeric'}, {'scalar', 'finite', '>', startTime});
            validateattributes(agvId, {'numeric'}, {'scalar', 'integer', '>=', 0});

            nodeIndex = double(nodeIndex(:))';
            window = struct( ...
                'edgeIndex', [nodeIndex, nodeIndex], ...
                'nodeIndex', nodeIndex, ...
                'startTime', double(startTime), ...
                'endTime', double(endTime), ...
                'agvId', double(agvId), ...
                'direction', -1, ...
                'windowType', 'node');
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

            if all(isfield(window, {'nodeIndex', 'windowType'})) && ...
                    numel(window.edgeIndex) == 4 && numel(window.nodeIndex) == 2 && ...
                    isscalar(window.startTime) && isscalar(window.endTime) && ...
                    isscalar(window.agvId) && isscalar(window.direction)
                window.edgeIndex = double(window.edgeIndex(:))';
                window.nodeIndex = double(window.nodeIndex(:))';
                window.startTime = double(window.startTime);
                window.endTime = double(window.endTime);
                window.agvId = double(window.agvId);
                window.direction = double(window.direction);
                window.windowType = char(string(window.windowType));
                return;
            end

            if isfield(window, 'windowType') && strcmp(char(string(window.windowType)), 'node')
                if isfield(window, 'nodeIndex') && ~isempty(window.nodeIndex)
                    nodeIndex = window.nodeIndex;
                else
                    nodeIndex = window.edgeIndex(1:2);
                end
                window = timewindow.TimeWindowManager.buildNodeWindow( ...
                    nodeIndex, window.startTime, window.endTime, window.agvId);
            else
                window = timewindow.TimeWindowManager.buildWindow( ...
                    window.edgeIndex, window.startTime, window.endTime, window.agvId, window.direction);
            end
        end

        function windows = normalizeWindowArray(windows)
            %NORMALIZEWINDOWARRAY Normalize every entry in a window array.
            if isempty(windows)
                windows = timewindow.TimeWindowManager.emptyWindowArray();
                return;
            end

            normalized = timewindow.TimeWindowManager.emptyWindowArray();
            normalized(numel(windows), 1) = timewindow.TimeWindowManager.normalizeWindow(windows(end));
            for i = 1:numel(windows)
                normalized(i, 1) = timewindow.TimeWindowManager.normalizeWindow(windows(i));
            end
            windows = normalized;
        end

        function indexMap = buildWindowIndex(windows)
            %BUILDWINDOWINDEX Group edge/node windows by resource and sort by time.
            indexMap = timewindow.TimeWindowManager.emptyWindowIndex();
            for i = 1:numel(windows)
                window = timewindow.TimeWindowManager.normalizeWindow(windows(i));
                key = timewindow.TimeWindowManager.getWindowResourceKey(window);
                if isKey(indexMap, key)
                    indexedWindows = indexMap(key);
                    indexedWindows(end + 1, 1) = window; %#ok<AGROW>
                else
                    indexedWindows = window;
                end
                indexMap(key) = indexedWindows;
            end

            keyList = indexMap.keys;
            for i = 1:numel(keyList)
                indexMap(keyList{i}) = timewindow.TimeWindowManager.sortWindowsByTime(indexMap(keyList{i}));
            end
        end

        function indexMap = insertWindowIntoIndex(indexMap, window)
            %INSERTWINDOWINTOINDEX Insert one normalized window into an index.
            window = timewindow.TimeWindowManager.normalizeWindow(window);
            key = timewindow.TimeWindowManager.getWindowResourceKey(window);
            if isKey(indexMap, key)
                indexedWindows = indexMap(key);
                indexedWindows(end + 1, 1) = window; %#ok<AGROW>
            else
                indexedWindows = window;
            end

            indexMap(key) = timewindow.TimeWindowManager.sortWindowsByTime(indexedWindows);
        end

        function indexMap = insertWindowsIntoIndex(indexMap, windows)
            %INSERTWINDOWSINTOINDEX Insert multiple windows into an index.
            for i = 1:numel(windows)
                indexMap = timewindow.TimeWindowManager.insertWindowIntoIndex(indexMap, windows(i));
            end
        end

        function [hasConflict, existingWindow] = findConflictInIndex(newWindow, indexMap)
            %FINDCONFLICTININDEX Check a candidate against same resource windows.
            hasConflict = false;
            existingWindow = timewindow.TimeWindowManager.emptyWindowArray();
            if isempty(indexMap)
                return;
            end

            newWindow = timewindow.TimeWindowManager.normalizeWindow(newWindow);
            key = timewindow.TimeWindowManager.getWindowResourceKey(newWindow);
            if ~isKey(indexMap, key)
                return;
            end

            indexedWindows = indexMap(key);
            for i = 1:numel(indexedWindows)
                existing = indexedWindows(i);
                if existing.startTime >= newWindow.endTime
                    break;
                end
                if existing.endTime <= newWindow.startTime
                    continue;
                end
                if existing.agvId > 0 && existing.agvId == newWindow.agvId
                    continue;
                end

                hasConflict = true;
                existingWindow = existing;
                return;
            end
        end

        function windows = sortWindowsByTime(windows)
            %SORTWINDOWSBYTIME Sort windows by start time, then end time.
            if numel(windows) <= 1
                return;
            end

            sortMatrix = [[windows.startTime].', [windows.endTime].', [windows.agvId].'];
            [~, order] = sortrows(sortMatrix, [1, 2, 3]);
            windows = windows(order);
        end

        function key = getDirectedEdgeKey(edgeIndex)
            %GETDIRECTEDEDGEKEY Convert a directed edge into a stable key string.
            edgeIndex = double(edgeIndex(:))';
            key = sprintf('%d_%d_%d_%d', edgeIndex(1), edgeIndex(2), edgeIndex(3), edgeIndex(4));
        end

        function key = getUndirectedEdgeKey(edgeIndex)
            %GETUNDIRECTEDEDGEKEY Convert an edge into a direction-free key string.
            edgeIndex = double(edgeIndex(:))';
            startNode = edgeIndex(1:2);
            endNode = edgeIndex(3:4);

            if endNode(1) < startNode(1) || ...
                    (endNode(1) == startNode(1) && endNode(2) < startNode(2))
                edgeIndex = [endNode, startNode];
            end

            key = timewindow.TimeWindowManager.getDirectedEdgeKey(edgeIndex);
        end

        function key = getNodeKey(nodeIndex)
            %GETNODEKEY Convert a node coordinate into a stable key string.
            nodeIndex = double(nodeIndex(:))';
            key = sprintf('%d_%d', nodeIndex(1), nodeIndex(2));
        end

        function key = getWindowResourceKey(window)
            %GETWINDOWRESOURCEKEY Return the conflict resource key for a window.
            if ~isfield(window, 'windowType')
                window = timewindow.TimeWindowManager.normalizeWindow(window);
            end
            if strcmp(window.windowType, 'node')
                key = ['node:', timewindow.TimeWindowManager.getNodeKey(window.nodeIndex)];
            else
                key = ['edge:', timewindow.TimeWindowManager.getUndirectedEdgeKey(window.edgeIndex)];
            end
        end

        function keySet = edgeKeySet(edgeList, useUndirectedKey)
            %EDGEKEYSET Convert an edge list into a lookup map of edge keys.
            if nargin < 2 || isempty(useUndirectedKey)
                useUndirectedKey = false;
            end

            keySet = containers.Map('KeyType', 'char', 'ValueType', 'logical');
            for i = 1:size(edgeList, 1)
                if useUndirectedKey
                    key = timewindow.TimeWindowManager.getUndirectedEdgeKey(edgeList(i, :));
                else
                    key = timewindow.TimeWindowManager.getDirectedEdgeKey(edgeList(i, :));
                end
                keySet(key) = true;
            end
        end

        function keySet = nodeKeySet(path)
            %NODEKEYSET Convert path nodes into a lookup map of node keys.
            keySet = containers.Map('KeyType', 'char', 'ValueType', 'logical');
            for i = 1:size(path, 1)
                keySet(timewindow.TimeWindowManager.getNodeKey(path(i, :))) = true;
            end
        end

        function tf = windowMatchesPath(window, edgeKeySet, nodeKeySet)
            %WINDOWMATCHESPATH Return true when a window belongs to a path.
            window = timewindow.TimeWindowManager.normalizeWindow(window);
            if strcmp(window.windowType, 'node')
                tf = isKey(nodeKeySet, timewindow.TimeWindowManager.getNodeKey(window.nodeIndex));
            else
                tf = isKey(edgeKeySet, timewindow.TimeWindowManager.getDirectedEdgeKey(window.edgeIndex));
            end
        end

        function window = cloneWindowWithTiming(window, startTime, endTime, agvId)
            %CLONEWINDOWWITHTIMING Copy a window resource with new timing/owner.
            window = timewindow.TimeWindowManager.normalizeWindow(window);
            if strcmp(window.windowType, 'node')
                window = timewindow.TimeWindowManager.buildNodeWindow( ...
                    window.nodeIndex, startTime, endTime, agvId);
            else
                window = timewindow.TimeWindowManager.buildWindow( ...
                    window.edgeIndex, startTime, endTime, agvId, window.direction);
            end
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
