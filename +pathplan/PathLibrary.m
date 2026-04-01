classdef PathLibrary
    %PATHLIBRARY Generate and store multiple candidate paths for tasks.
    %   Candidate paths are generated from A* and diversified by blocking
    %   selected interior nodes from previously discovered routes.

    methods (Static)
        function libraryEntry = generateLibrary(mapObj, taskObj, numPaths, speed, agvId)
            %GENERATELIBRARY Generate up to numPaths candidate paths for a task.
            % Inputs:
            %   mapObj   - map.MapClass instance.
            %   taskObj  - task.TaskClass instance.
            %   numPaths - Requested number of candidate paths.
            %   speed    - Speed used to derive time windows.
            %   agvId    - Optional reservation owner id for time windows.
            % Output:
            %   libraryEntry - Struct containing candidate path data.
            if nargin < 3 || isempty(numPaths)
                numPaths = 3;
            end
            if nargin < 4 || isempty(speed)
                speed = 1.0;
            end
            if nargin < 5 || isempty(agvId)
                agvId = taskObj.id;
            end

            validateattributes(numPaths, {'numeric'}, {'scalar', 'integer', 'positive'});
            validateattributes(speed, {'numeric'}, {'scalar', 'positive', 'finite'});

            anchorNodes = [taskObj.start; taskObj.getWaypointPositions()];
            candidatePaths = repmat(pathplan.PathLibrary.emptyCandidatePath(), 0, 1);
            candidateSignatures = {};
            visitedAvoidSets = {};
            avoidQueue = {zeros(0, 2)};

            while ~isempty(avoidQueue) && numel(candidatePaths) < numPaths
                avoidNodes = avoidQueue{1};
                avoidQueue(1) = [];

                avoidSignature = pathplan.PathLibrary.nodeMatrixSignature(avoidNodes);
                if any(strcmp(visitedAvoidSets, avoidSignature))
                    continue;
                end
                visitedAvoidSets{end + 1} = avoidSignature; %#ok<AGROW>

                route = pathplan.PathLibrary.buildTaskRoute(mapObj, taskObj, avoidNodes, agvId);
                if isempty(route)
                    continue;
                end

                routeSignature = pathplan.PathLibrary.nodeMatrixSignature(route);
                if ~any(strcmp(candidateSignatures, routeSignature))
                    candidatePaths(end + 1, 1) = pathplan.PathLibrary.buildCandidatePath( ... %#ok<AGROW>
                        route, speed, agvId, avoidNodes, numel(candidatePaths) + 1);
                    candidateSignatures{end + 1} = routeSignature; %#ok<AGROW>
                end

                diversionNodes = pathplan.PathLibrary.selectDiversionNodes(route, anchorNodes);
                for i = 1:size(diversionNodes, 1)
                    nextAvoidNodes = pathplan.PathLibrary.uniqueRows([avoidNodes; diversionNodes(i, :)]);
                    nextSignature = pathplan.PathLibrary.nodeMatrixSignature(nextAvoidNodes);
                    if ~any(strcmp(visitedAvoidSets, nextSignature))
                        avoidQueue{end + 1} = nextAvoidNodes; %#ok<AGROW>
                    end
                end
            end

            libraryEntry = struct( ...
                'taskId', taskObj.id, ...
                'start', taskObj.start, ...
                'waypoints', taskObj.waypoints, ...
                'numRequested', double(numPaths), ...
                'numGenerated', double(numel(candidatePaths)), ...
                'speed', double(speed), ...
                'paths', candidatePaths);
        end

        function libraryData = generateForTasks(mapObj, tasks, numPaths, speed)
            %GENERATEFORTASKS Generate path library entries for many tasks.
            if nargin < 3 || isempty(numPaths)
                numPaths = 3;
            end
            if nargin < 4 || isempty(speed)
                speed = 1.0;
            end

            if isempty(tasks)
                libraryData = repmat(struct(), 0, 1);
                return;
            end

            firstEntry = pathplan.PathLibrary.generateLibrary(mapObj, tasks(1), numPaths, speed, tasks(1).id);
            libraryData = repmat(firstEntry, numel(tasks), 1);
            for i = 2:numel(tasks)
                libraryData(i, 1) = pathplan.PathLibrary.generateLibrary(mapObj, tasks(i), numPaths, speed, tasks(i).id);
            end
        end

        function saveLibrary(outputPath, libraryData)
            %SAVELIBRARY Save generated path library data to disk.
            pathLibraryData = libraryData; %#ok<NASGU>
            save(outputPath, 'pathLibraryData');
        end
    end

    methods (Static, Access = private)
        function candidate = buildCandidatePath(route, speed, agvId, avoidNodes, pathId)
            %BUILDCANDIDATEPATH Package one concrete route with metadata.
            manager = timewindow.TimeWindowManager();
            [timeWindows, conflictInfo] = manager.reservePath(agvId, route, 0.0, speed);
            if ~isempty(conflictInfo)
                error('PathLibrary:UnexpectedReservationConflict', ...
                    'Unexpected conflict while computing time windows for a standalone path.');
            end

            candidate = struct( ...
                'pathId', double(pathId), ...
                'nodes', double(route), ...
                'timeWindows', timeWindows, ...
                'length', double(pathplan.PathLibrary.computePathLength(route)), ...
                'blockedNodesUsed', double(avoidNodes));
        end

        function route = buildTaskRoute(mapObj, taskObj, avoidNodes, agvId)
            %BUILDTASKROUTE Plan the full multi-waypoint route for a task.
            anchors = [taskObj.start; taskObj.getWaypointPositions()];
            route = zeros(0, 2);

            for segmentIdx = 1:(size(anchors, 1) - 1)
                startNode = anchors(segmentIdx, :);
                goalNode = anchors(segmentIdx + 1, :);
                segmentPath = pathplan.PathLibrary.planSegment( ...
                    mapObj, startNode, goalNode, taskObj.id, avoidNodes, agvId);

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

        function segmentPath = planSegment(mapObj, startNode, goalNode, taskId, avoidNodes, agvId)
            %PLANSEGMENT Plan one task segment on a cloned working map.
            workingMap = pathplan.PathLibrary.cloneMap(mapObj);
            pathplan.PathLibrary.applyAvoidNodes(workingMap, avoidNodes, startNode, goalNode);
            workingMap.registerTaskTarget(taskId, pathplan.PathLibrary.uniqueRows([startNode; goalNode]));
            segmentPath = pathplan.AStar(workingMap, startNode, goalNode, agvId, taskId);
        end

        function workingMap = cloneMap(mapObj)
            %CLONEMAP Copy map geometry, occupancy, and task-target metadata.
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

        function applyAvoidNodes(mapObj, avoidNodes, startNode, goalNode)
            %APPLYAVOIDNODES Temporarily block selected nodes for diversification.
            for i = 1:size(avoidNodes, 1)
                node = avoidNodes(i, :);
                if isequal(node, startNode) || isequal(node, goalNode)
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

        function lengthValue = computePathLength(route)
            %COMPUTEPATHLENGTH Compute Euclidean route length for reporting.
            if size(route, 1) < 2
                lengthValue = 0.0;
                return;
            end

            deltas = diff(route, 1, 1);
            lengthValue = sum(sqrt(sum(deltas .^ 2, 2)));
        end

        function nodes = selectDiversionNodes(route, anchorNodes)
            %SELECTDIVERSIONNODES Choose interior nodes to spawn route variants.
            if size(route, 1) <= 2
                nodes = zeros(0, 2);
                return;
            end

            interiorNodes = route(2:end-1, :);
            keepMask = true(size(interiorNodes, 1), 1);
            for i = 1:size(interiorNodes, 1)
                if any(all(anchorNodes == interiorNodes(i, :), 2))
                    keepMask(i) = false;
                end
            end

            interiorNodes = interiorNodes(keepMask, :);
            interiorNodes = pathplan.PathLibrary.uniqueRows(interiorNodes);
            if isempty(interiorNodes)
                nodes = zeros(0, 2);
                return;
            end

            sampleCount = min(6, size(interiorNodes, 1));
            sampleIdx = unique(round(linspace(1, size(interiorNodes, 1), sampleCount)));
            nodes = interiorNodes(sampleIdx, :);
        end

        function matrix = uniqueRows(matrix)
            %UNIQUEROWS Remove duplicate rows while preserving order.
            if isempty(matrix)
                return;
            end

            [~, uniqueIdx] = unique(matrix, 'rows', 'stable');
            matrix = matrix(sort(uniqueIdx), :);
        end

        function signature = nodeMatrixSignature(matrix)
            %NODEMATRIXSIGNATURE Convert a node matrix into a dedupe key.
            if isempty(matrix)
                signature = '[]';
            else
                signature = mat2str(double(matrix));
            end
        end

        function candidate = emptyCandidatePath()
            %EMPTYCANDIDATEPATH Return a default candidate-path struct.
            candidate = struct( ...
                'pathId', 0, ...
                'nodes', zeros(0, 2), ...
                'timeWindows', timewindow.TimeWindowManager.emptyWindowArray(), ...
                'length', 0.0, ...
                'blockedNodesUsed', zeros(0, 2));
        end
    end
end
