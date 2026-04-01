function path = AStar(mapInput, startPos, goalPos, agvId, taskId)
%ASTAR Compute a 4-neighbor A* path on the warehouse grid.
% Inputs:
%   mapInput  - map.MapClass object or numeric grid.
%   startPos  - 1-by-2 start coordinate [row, col].
%   goalPos   - 1-by-2 goal coordinate [row, col].
%   agvId     - AGV identifier for occupancy-aware traversal.
%   taskId    - Task identifier for target shelf unlocking.
% Output:
%   path      - N-by-2 path coordinates from start to goal. Returns an
%               empty matrix when no feasible path exists.

if nargin < 4
    agvId = [];
end
if nargin < 5
    taskId = [];
end

validateattributes(startPos, {'numeric'}, {'vector', 'numel', 2, 'finite', 'positive'});
validateattributes(goalPos, {'numeric'}, {'vector', 'numel', 2, 'finite', 'positive'});

startPos = double(startPos(:))';
goalPos = double(goalPos(:))';

if any(mod([startPos, goalPos], 1) ~= 0)
    error('AStar:InvalidCoordinate', 'Start and goal coordinates must be integer grid positions.');
end

[gridSize, isPassableFn] = resolveMapAccess(mapInput);

if ~isInside(startPos, gridSize) || ~isInside(goalPos, gridSize)
    error('AStar:OutOfBounds', 'Start or goal position is outside the map bounds.');
end

if ~isPassableFn(startPos(1), startPos(2), agvId, taskId) || ...
        ~isPassableFn(goalPos(1), goalPos(2), agvId, taskId)
    path = zeros(0, 2);
    return;
end

if isequal(startPos, goalPos)
    path = startPos;
    return;
end

numRows = gridSize(1);
numCols = gridSize(2);
numNodes = numRows * numCols;

gScore = inf(numNodes, 1);
fScore = inf(numNodes, 1);
cameFrom = zeros(numNodes, 1);
openSet = false(numNodes, 1);
closedSet = false(numNodes, 1);

startIdx = sub2ind([numRows, numCols], startPos(1), startPos(2));
goalIdx = sub2ind([numRows, numCols], goalPos(1), goalPos(2));

startHeuristic = manhattanDistance(startPos, goalPos);
gScore(startIdx) = 0;
fScore(startIdx) = improvedCost(0, startPos, startPos, goalPos);
openSet(startIdx) = true;

neighborOffsets = [-1, 0; 1, 0; 0, -1; 0, 1];

while any(openSet)
    openIndices = find(openSet);
    [~, bestLocalIdx] = min(fScore(openIndices));
    currentIdx = openIndices(bestLocalIdx);

    if currentIdx == goalIdx
        path = reconstructPath(cameFrom, currentIdx, numRows, numCols);
        return;
    end

    openSet(currentIdx) = false;
    closedSet(currentIdx) = true;
    [currentRow, currentCol] = ind2sub([numRows, numCols], currentIdx);
    currentPos = [currentRow, currentCol];

    for i = 1:size(neighborOffsets, 1)
        neighborPos = currentPos + neighborOffsets(i, :);
        if ~isInside(neighborPos, gridSize)
            continue;
        end

        neighborIdx = sub2ind([numRows, numCols], neighborPos(1), neighborPos(2));
        if closedSet(neighborIdx)
            continue;
        end

        if ~isPassableFn(neighborPos(1), neighborPos(2), agvId, taskId)
            continue;
        end

        tentativeG = gScore(currentIdx) + 1;
        if tentativeG >= gScore(neighborIdx)
            continue;
        end

        cameFrom(neighborIdx) = currentIdx;
        gScore(neighborIdx) = tentativeG;
        fScore(neighborIdx) = improvedCost(tentativeG, startPos, neighborPos, goalPos);
        openSet(neighborIdx) = true;
    end
end

path = zeros(0, 2);
end

function [gridSize, isPassableFn] = resolveMapAccess(mapInput)
if isa(mapInput, 'map.MapClass')
    gridSize = size(mapInput.grid);
    isPassableFn = @(row, col, agvId, taskId) mapInput.isPassable(row, col, agvId, taskId);
    return;
end

validateattributes(mapInput, {'numeric'}, {'2d', 'nonempty', 'finite'});
gridData = double(mapInput);
gridSize = size(gridData);
isPassableFn = @(row, col, ~, ~) ismember(gridData(row, col), [0, 2]);
end

function tf = isInside(position, gridSize)
tf = position(1) >= 1 && position(1) <= gridSize(1) && ...
    position(2) >= 1 && position(2) <= gridSize(2);
end

function distance = manhattanDistance(a, b)
distance = abs(a(1) - b(1)) + abs(a(2) - b(2));
end

function score = improvedCost(gCost, startPos, currentPos, goalPos)
hCost = manhattanDistance(currentPos, goalPos);
referenceDistance = manhattanDistance(startPos, goalPos);
remainingDistance = hCost;

if remainingDistance <= 0
    coefficient = 1.0;
else
    coefficient = 1.0 + referenceDistance / remainingDistance;
end

score = gCost + coefficient * hCost;
end

function path = reconstructPath(cameFrom, currentIdx, numRows, numCols)
indices = currentIdx;
while cameFrom(currentIdx) ~= 0
    currentIdx = cameFrom(currentIdx);
    indices = [currentIdx; indices]; %#ok<AGROW>
end

path = zeros(numel(indices), 2);
for i = 1:numel(indices)
    [row, col] = ind2sub([numRows, numCols], indices(i));
    path(i, :) = [row, col];
end
end
