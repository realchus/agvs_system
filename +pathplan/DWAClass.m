classdef DWAClass < handle
    %DWACLASS Lightweight dynamic-window local planner for grid AGVs.
    %   The planner samples short-horizon controls around the current AGV
    %   state, evaluates them against dynamic obstacles and the remaining
    %   global path, then reconnects to the route with A*.

    properties
        minSpeed
        maxSpeed
        speedSamples
        lookaheadSteps
        safetyDistance
        speedWeight
        clearanceWeight
        headingWeight
        pathWeight
    end

    methods
        function obj = DWAClass(params)
            %DWACLASS Construct a planner with optional tuning parameters.
            if nargin < 1
                params = struct();
            end

            obj.minSpeed = pathplan.DWAClass.paramOrDefault(params, 'minSpeed', 0.5);
            obj.maxSpeed = pathplan.DWAClass.paramOrDefault(params, 'maxSpeed', 1.0);
            obj.speedSamples = pathplan.DWAClass.paramOrDefault(params, 'speedSamples', 3);
            obj.lookaheadSteps = pathplan.DWAClass.paramOrDefault(params, 'lookaheadSteps', 3);
            obj.safetyDistance = pathplan.DWAClass.paramOrDefault(params, 'safetyDistance', 1.0);
            obj.speedWeight = pathplan.DWAClass.paramOrDefault(params, 'speedWeight', 0.25);
            obj.clearanceWeight = pathplan.DWAClass.paramOrDefault(params, 'clearanceWeight', 0.35);
            obj.headingWeight = pathplan.DWAClass.paramOrDefault(params, 'headingWeight', 0.25);
            obj.pathWeight = pathplan.DWAClass.paramOrDefault(params, 'pathWeight', 0.15);
        end

        function [localPath, planningInfo] = planLocalPath(obj, agvObj, mapObj, obstacles, globalPath, taskId)
            %PLANLOCALPATH Generate a local detour and reconnect to the route.
            % Inputs:
            %   agvObj     - AGV object requiring avoidance.
            %   mapObj     - map.MapClass instance.
            %   obstacles  - N-by-2 dynamic obstacle positions.
            %   globalPath - Remaining global path beginning at current node.
            %   taskId     - Optional task identifier for target access.
            % Outputs:
            %   localPath    - Path that locally avoids the obstacle.
            %   planningInfo - Struct describing the chosen control.
            if nargin < 6
                taskId = [];
            end

            currentNode = round(double(agvObj.position(:))');
            obstacles = pathplan.DWAClass.uniqueRows(double(obstacles));
            globalPath = double(globalPath);

            localPath = zeros(0, 2);
            planningInfo = struct( ...
                'triggered', false, ...
                'control', struct('speed', 0.0, 'direction', [0, 0]), ...
                'score', -inf, ...
                'reconnectGoal', zeros(1, 2), ...
                'candidateCount', 0);

            if isempty(globalPath)
                return;
            end
            if ~isequal(globalPath(1, :), currentNode)
                globalPath = [currentNode; globalPath];
            end
            if size(globalPath, 1) < 2
                localPath = globalPath;
                return;
            end

            [obstacleAhead, obstacleIndex] = obj.detectObstacleAhead(globalPath, obstacles);
            if ~obstacleAhead
                localPath = globalPath;
                return;
            end

            planningInfo.triggered = true;
            workingMap = pathplan.DWAClass.cloneMap(mapObj);
            pathplan.DWAClass.applyObstacleNodes(workingMap, obstacles, currentNode);

            reconnectGoal = obj.selectReconnectGoal(globalPath, obstacles, obstacleIndex);
            controls = obj.sampleControls(agvObj);
            bestScore = -inf;
            bestPath = zeros(0, 2);
            bestControl = struct('speed', 0.0, 'direction', [0, 0]);

            for i = 1:numel(controls)
                nextNode = currentNode + controls(i).direction;
                planningInfo.candidateCount = planningInfo.candidateCount + 1;

                if ~pathplan.DWAClass.isGridMoveValid(workingMap, nextNode, agvObj.id, taskId)
                    continue;
                end
                if pathplan.DWAClass.containsRow(obstacles, nextNode)
                    continue;
                end

                connector = pathplan.AStar(workingMap, nextNode, reconnectGoal, agvObj.id, taskId);
                if isempty(connector)
                    continue;
                end

                suffix = obj.suffixFromGoal(globalPath, reconnectGoal);
                candidatePath = [currentNode; connector];
                if ~isempty(suffix)
                    candidatePath = [candidatePath; suffix(2:end, :)]; %#ok<AGROW>
                end
                candidatePath = pathplan.DWAClass.removeSequentialDuplicates(candidatePath);

                score = obj.scoreCandidate(candidatePath, controls(i), obstacles, globalPath, reconnectGoal);
                if score > bestScore
                    bestScore = score;
                    bestPath = candidatePath;
                    bestControl = controls(i);
                end
            end

            localPath = bestPath;
            planningInfo.control = bestControl;
            planningInfo.score = bestScore;
            planningInfo.reconnectGoal = reconnectGoal;
        end
    end

    methods (Access = private)
        function [tf, obstacleIndex] = detectObstacleAhead(obj, globalPath, obstacles)
            %DETECTOBSTACLEAHEAD Check whether the near-future route is blocked.
            tf = false;
            obstacleIndex = 0;
            lookaheadCount = min(size(globalPath, 1), obj.lookaheadSteps + 1);
            for i = 2:lookaheadCount
                if pathplan.DWAClass.containsRow(obstacles, globalPath(i, :))
                    tf = true;
                    obstacleIndex = i;
                    return;
                end
            end
        end

        function reconnectGoal = selectReconnectGoal(~, globalPath, obstacles, obstacleIndex)
            %SELECTRECONNECTGOAL Pick the next reachable node after the blockage.
            reconnectGoal = globalPath(end, :);
            startIdx = min(size(globalPath, 1), max(2, obstacleIndex + 1));
            for i = startIdx:size(globalPath, 1)
                if ~pathplan.DWAClass.containsRow(obstacles, globalPath(i, :))
                    reconnectGoal = globalPath(i, :);
                    return;
                end
            end
        end

        function controls = sampleControls(obj, agvObj)
            %SAMPLECONTROLS Enumerate short-horizon velocity and heading samples.
            speedValues = linspace(obj.minSpeed, obj.maxSpeed, obj.speedSamples);
            headingSequence = pathplan.DWAClass.headingPriority(agvObj);
            controls = repmat(struct('speed', 0.0, 'direction', [0, 0]), ...
                numel(speedValues) * size(headingSequence, 1), 1);

            controlIndex = 1;
            for headingIdx = 1:size(headingSequence, 1)
                for speedIdx = 1:numel(speedValues)
                    controls(controlIndex) = struct( ...
                        'speed', double(speedValues(speedIdx)), ...
                        'direction', double(headingSequence(headingIdx, :)));
                    controlIndex = controlIndex + 1;
                end
            end
        end

        function suffix = suffixFromGoal(~, globalPath, reconnectGoal)
            %SUFFIXFROMGOAL Return the remaining route after the reconnect node.
            suffix = reconnectGoal;
            for i = 1:size(globalPath, 1)
                if isequal(globalPath(i, :), reconnectGoal)
                    suffix = globalPath(i:end, :);
                    return;
                end
            end
        end

        function score = scoreCandidate(obj, candidatePath, control, obstacles, globalPath, reconnectGoal)
            %SCORECANDIDATE Evaluate one locally avoided path candidate.
            localPrefix = candidatePath(1:min(end, obj.lookaheadSteps + 1), :);
            clearance = obj.computeClearance(localPrefix, obstacles);
            if isinf(clearance)
                clearance = obj.safetyDistance + 1.0;
            end

            finalGoal = globalPath(end, :);
            headingScore = -pathplan.DWAClass.manhattanDistance(candidatePath(end, :), finalGoal);
            reconnectScore = -pathplan.DWAClass.manhattanDistance(candidatePath(min(end, 2), :), reconnectGoal);
            pathAlignment = -pathplan.DWAClass.pathDeviation(candidatePath, globalPath);
            speedScore = control.speed / max(obj.maxSpeed, eps);

            score = obj.speedWeight * speedScore + ...
                obj.clearanceWeight * clearance + ...
                obj.headingWeight * (headingScore + reconnectScore) + ...
                obj.pathWeight * pathAlignment;
        end

        function clearance = computeClearance(~, pathNodes, obstacles)
            %COMPUTECLEARANCE Measure the closest obstacle distance along a prefix.
            if isempty(obstacles)
                clearance = inf;
                return;
            end

            clearance = inf;
            for i = 1:size(pathNodes, 1)
                delta = obstacles - pathNodes(i, :);
                distances = sqrt(sum(delta .^ 2, 2));
                clearance = min(clearance, min(distances));
            end
        end
    end

    methods (Static)
        function value = paramOrDefault(params, fieldName, defaultValue)
            %PARAMORDEFAULT Read a parameter from a struct.
            if isstruct(params) && isfield(params, fieldName) && ~isempty(params.(fieldName))
                value = params.(fieldName);
            else
                value = defaultValue;
            end
        end

        function workingMap = cloneMap(mapObj)
            %CLONEMAP Clone a map including occupancy and target metadata.
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

        function applyObstacleNodes(mapObj, obstacles, protectedNode)
            %APPLYOBSTACLENODES Mark dynamic obstacles as temporarily blocked.
            for i = 1:size(obstacles, 1)
                node = obstacles(i, :);
                if isequal(node, protectedNode)
                    continue;
                end
                if node(1) < 1 || node(1) > size(mapObj.baseGrid, 1) || ...
                        node(2) < 1 || node(2) > size(mapObj.baseGrid, 2)
                    continue;
                end

                mapObj.baseGrid(node(1), node(2)) = 3;
                if mapObj.grid(node(1), node(2)) ~= 2
                    mapObj.grid(node(1), node(2)) = 3;
                end
            end
        end

        function tf = isGridMoveValid(mapObj, node, agvId, taskId)
            %ISGRIDMOVEVALID Return true when a sampled node is traversable.
            tf = node(1) >= 1 && node(1) <= size(mapObj.baseGrid, 1) && ...
                node(2) >= 1 && node(2) <= size(mapObj.baseGrid, 2) && ...
                mapObj.isPassable(node(1), node(2), agvId, taskId);
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

        function distance = manhattanDistance(a, b)
            %MANHATTANDISTANCE Compute Manhattan distance.
            distance = abs(double(a(1)) - double(b(1))) + abs(double(a(2)) - double(b(2)));
        end

        function deviation = pathDeviation(candidatePath, globalPath)
            %PATHDEVIATION Compute aggregate distance from local path to route.
            deviation = 0.0;
            for i = 1:size(candidatePath, 1)
                delta = globalPath - candidatePath(i, :);
                deviation = deviation + min(sum(abs(delta), 2));
            end
        end

        function headings = headingPriority(agvObj)
            %HEADINGPRIORITY Return preferred local search directions.
            if isempty(agvObj.path) || agvObj.pathIndex > size(agvObj.path, 1)
                headings = [-1, 0; 1, 0; 0, 1; 0, -1];
                return;
            end

            currentNode = round(double(agvObj.position(:))');
            nextNode = round(double(agvObj.path(agvObj.pathIndex, :)));
            forward = nextNode - currentNode;

            if isequal(forward, [0, 1])
                headings = [-1, 0; 1, 0; 0, 1; 0, -1];
            elseif isequal(forward, [0, -1])
                headings = [-1, 0; 1, 0; 0, -1; 0, 1];
            elseif isequal(forward, [1, 0])
                headings = [0, 1; 0, -1; 1, 0; -1, 0];
            elseif isequal(forward, [-1, 0])
                headings = [0, 1; 0, -1; -1, 0; 1, 0];
            else
                headings = [-1, 0; 1, 0; 0, 1; 0, -1];
            end
        end
    end
end
