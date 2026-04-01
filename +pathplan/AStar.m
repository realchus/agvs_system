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
closedSet = false(numNodes, 1);

startIdx = sub2ind([numRows, numCols], startPos(1), startPos(2));
goalIdx = sub2ind([numRows, numCols], goalPos(1), goalPos(2));

gScore(startIdx) = 0;
fScore(startIdx) = improvedCost(0, startPos, startPos, goalPos);
heap = zeros(max(16, min(numNodes, 64)), 2);
heapSize = 0;
[heap, heapSize] = heapPush(heap, heapSize, startIdx, fScore(startIdx));

neighborOffsets = [-1, 0; 1, 0; 0, -1; 0, 1];

while heapSize > 0
    [heap, heapSize, currentIdx, currentPriority] = heapPop(heap, heapSize);
    if closedSet(currentIdx) || currentPriority > fScore(currentIdx) + eps
        continue;
    end

    if currentIdx == goalIdx
        path = reconstructPath(cameFrom, currentIdx, numRows, numCols);
        return;
    end

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
        [heap, heapSize] = heapPush(heap, heapSize, neighborIdx, fScore(neighborIdx));
    end
end

path = zeros(0, 2);
end

function [gridSize, isPassableFn] = resolveMapAccess(mapInput)
%RESOLVEMAPACCESS Normalize a map object or raw grid into access helpers.
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
%ISINSIDE Return true when a grid position is inside the map bounds.
tf = position(1) >= 1 && position(1) <= gridSize(1) && ...
    position(2) >= 1 && position(2) <= gridSize(2);
end

function distance = manhattanDistance(a, b)
%MANHATTANDISTANCE Compute 4-neighbor Manhattan distance.
distance = abs(a(1) - b(1)) + abs(a(2) - b(2));
end

function score = improvedCost(gCost, startPos, currentPos, goalPos)
%IMPROVEDCOST Evaluate the weighted heuristic requested by the project.
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
%RECONSTRUCTPATH Backtrack the predecessor chain into a node sequence.
pathLength = 1;
traceIdx = currentIdx;
while cameFrom(traceIdx) ~= 0
    traceIdx = cameFrom(traceIdx);
    pathLength = pathLength + 1;
end

path = zeros(pathLength, 2);
writeIdx = pathLength;
traceIdx = currentIdx;
while true
    [row, col] = ind2sub([numRows, numCols], traceIdx);
    path(writeIdx, :) = [row, col];
    predecessor = cameFrom(traceIdx);
    if predecessor == 0
        return;
    end
    traceIdx = predecessor;
    writeIdx = writeIdx - 1;
end
end

function [heap, heapSize] = heapPush(heap, heapSize, nodeIdx, priority)
%HEAPPUSH Insert a [node, priority] pair into the binary min-heap.
heapSize = heapSize + 1;
if heapSize > size(heap, 1)
    growth = max(16, size(heap, 1));
    heap(end + growth, :) = 0;
end

heap(heapSize, :) = [double(nodeIdx), double(priority)];
childIdx = heapSize;
while childIdx > 1
    parentIdx = floor(childIdx / 2);
    if ~heapLess(heap(childIdx, :), heap(parentIdx, :))
        break;
    end

    temp = heap(parentIdx, :);
    heap(parentIdx, :) = heap(childIdx, :);
    heap(childIdx, :) = temp;
    childIdx = parentIdx;
end
end

function [heap, heapSize, nodeIdx, priority] = heapPop(heap, heapSize)
%HEAPPOP Remove the lowest-priority entry from the binary min-heap.
nodeIdx = heap(1, 1);
priority = heap(1, 2);
heap(1, :) = heap(heapSize, :);
heap(heapSize, :) = 0;
heapSize = heapSize - 1;

parentIdx = 1;
while true
    leftIdx = parentIdx * 2;
    rightIdx = leftIdx + 1;
    smallestIdx = parentIdx;

    if leftIdx <= heapSize && heapLess(heap(leftIdx, :), heap(smallestIdx, :))
        smallestIdx = leftIdx;
    end
    if rightIdx <= heapSize && heapLess(heap(rightIdx, :), heap(smallestIdx, :))
        smallestIdx = rightIdx;
    end
    if smallestIdx == parentIdx
        break;
    end

    temp = heap(parentIdx, :);
    heap(parentIdx, :) = heap(smallestIdx, :);
    heap(smallestIdx, :) = temp;
    parentIdx = smallestIdx;
end
end

function tf = heapLess(entryA, entryB)
%HEAPLESS Compare heap entries by priority and then node index.
if entryA(2) ~= entryB(2)
    tf = entryA(2) < entryB(2);
else
    tf = entryA(1) < entryB(1);
end
end
