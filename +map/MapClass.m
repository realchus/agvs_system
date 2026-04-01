classdef MapClass < handle
    %MAPCLASS Warehouse grid map model with dynamic AGV occupancy support.
    %   The map stores a static base layout and overlays AGV occupancy on
    %   top of it. Cell values follow the project convention:
    %   0 = free lane, 1 = shelf, 2 = occupied by AGV, 3 = blocked area,
    %   4 = empty shelf location.

    properties
        baseGrid
        grid
        colors
        agvOccupancy
        taskTargets
    end

    methods
        function obj = MapClass(grid, colors)
            %MAPCLASS Construct a map model.
            % Inputs:
            %   grid   - Static grid layout matrix.
            %   colors - N-by-3 RGB matrix matching values 0..N-1.
            if nargin < 1 || isempty(grid)
                grid = map.MapClass.createDefaultGrid();
            end

            if nargin < 2 || isempty(colors)
                colors = map.MapClass.defaultColors();
            end

            obj.validateGrid(grid);

            obj.baseGrid = double(grid);
            obj.baseGrid(obj.baseGrid == 2) = 0;
            obj.grid = obj.baseGrid;
            obj.colors = double(colors);
            obj.agvOccupancy = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.taskTargets = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.syncGridWithOccupancy();
        end

        function tf = isPassable(obj, row, col, agvId, taskId)
            %ISPASSABLE Check whether the requested cell can be traversed.
            % Inputs:
            %   row, col - Grid coordinate.
            %   agvId    - AGV requesting the traversal.
            %   taskId   - Task identifier used to unlock a target shelf.
            % Output:
            %   tf       - True when the cell is traversable.
            if nargin < 4
                agvId = [];
            end
            if nargin < 5
                taskId = [];
            end

            if ~obj.isInside(row, col)
                tf = false;
                return;
            end

            occupiedBy = obj.getOccupyingAgv(row, col);
            if ~isempty(occupiedBy)
                tf = ~isempty(agvId) && occupiedBy == agvId;
                return;
            end

            cellValue = obj.baseGrid(row, col);
            switch cellValue
                case 0
                    tf = true;
                case {1, 3, 4}
                    tf = obj.isTaskTarget(row, col, taskId);
                otherwise
                    tf = false;
            end
        end

        function setAGVOccupancy(obj, agvId, row, col)
            %SETAGVOCCUPANCY Set or move an AGV occupancy marker.
            % Inputs:
            %   agvId    - AGV identifier.
            %   row, col - Target grid coordinate.
            if ~obj.isInside(row, col)
                error('MapClass:OutOfBounds', 'AGV position (%d, %d) is out of bounds.', row, col);
            end

            if ~obj.isPassable(row, col, agvId, [])
                error('MapClass:NotPassable', 'Cell (%d, %d) is not passable for AGV %d.', row, col, agvId);
            end

            if isKey(obj.agvOccupancy, agvId)
                remove(obj.agvOccupancy, agvId);
            end

            obj.agvOccupancy(agvId) = [row, col];
            obj.syncGridWithOccupancy();
        end

        function clearAGVOccupancy(obj, agvId)
            %CLEARAGVOCCUPANCY Remove an AGV occupancy marker.
            % Input:
            %   agvId - AGV identifier.
            if isKey(obj.agvOccupancy, agvId)
                remove(obj.agvOccupancy, agvId);
                obj.syncGridWithOccupancy();
            end
        end

        function registerTaskTarget(obj, taskId, positions)
            %REGISTERTASKTARGET Register shelf cells that may be opened for a task.
            % Inputs:
            %   taskId    - Task identifier.
            %   positions - N-by-2 matrix of [row, col] target positions.
            if isempty(positions)
                obj.taskTargets(taskId) = zeros(0, 2);
                return;
            end

            validateattributes(positions, {'numeric'}, {'2d', 'ncols', 2, 'positive', 'finite'});
            if any(mod(positions(:), 1) ~= 0)
                error('MapClass:InvalidTaskTarget', 'Task target positions must use integer grid coordinates.');
            end
            obj.taskTargets(taskId) = double(positions);
        end

        function clearTaskTarget(obj, taskId)
            %CLEARTASKTARGET Remove registered target cells for a task.
            % Input:
            %   taskId - Task identifier.
            if isKey(obj.taskTargets, taskId)
                remove(obj.taskTargets, taskId);
            end
        end

        function color = getColor(obj, row, col)
            %GETCOLOR Return the RGB color for a map cell.
            % Inputs:
            %   row, col - Grid coordinate.
            % Output:
            %   color    - 1-by-3 RGB vector.
            if ~obj.isInside(row, col)
                error('MapClass:OutOfBounds', 'Requested color at (%d, %d) is out of bounds.', row, col);
            end

            value = obj.grid(row, col);
            color = obj.colors(value + 1, :);
        end

        function plot(obj, ax)
            %PLOT Render the map using the configured color palette.
            % Input:
            %   ax - Optional target axes.
            if nargin < 2 || isempty(ax)
                figure('Color', 'w', 'Name', 'Warehouse Grid Map');
                ax = gca;
            end

            imagesc(ax, obj.grid);
            colormap(ax, obj.colors);
            axis(ax, 'equal');
            axis(ax, 'tight');
            title(ax, 'Warehouse Grid Map');
            set(ax, 'XTick', 0.5:1:size(obj.grid, 2) + 0.5, ...
                'YTick', 0.5:1:size(obj.grid, 1) + 0.5, ...
                'XTickLabel', [], 'YTickLabel', [], 'TickLength', [0 0], ...
                'GridLineStyle', '-', 'XGrid', 'on', 'YGrid', 'on', ...
                'GridColor', [0.7, 0.7, 0.7], 'GridAlpha', 0.4);
        end

        function config = toStruct(obj)
            %TOSTRUCT Export map data into a save-friendly struct.
            % Output:
            %   config - Struct containing map layout, colors and metadata.
            config = struct();
            config.baseGrid = obj.baseGrid;
            config.grid = obj.grid;
            config.colors = obj.colors;
            config.defaultAGVPositions = map.MapClass.defaultAGVPositions();
            config.occupancy = obj.exportMapEntries(obj.agvOccupancy, 'position');
            config.taskTargets = obj.exportMapEntries(obj.taskTargets, 'positions');
        end
    end

    methods (Access = private)
        function tf = isInside(obj, row, col)
            tf = row >= 1 && row <= size(obj.baseGrid, 1) && ...
                col >= 1 && col <= size(obj.baseGrid, 2);
        end

        function agvId = getOccupyingAgv(obj, row, col)
            agvId = [];
            keysList = obj.agvOccupancy.keys;
            for i = 1:numel(keysList)
                candidateId = keysList{i};
                position = obj.agvOccupancy(candidateId);
                if isequal(position, [row, col])
                    agvId = candidateId;
                    return;
                end
            end
        end

        function tf = isTaskTarget(obj, row, col, taskId)
            tf = false;
            if isempty(taskId) || ~isKey(obj.taskTargets, taskId)
                return;
            end

            positions = obj.taskTargets(taskId);
            tf = any(all(positions == [row, col], 2));
        end

        function syncGridWithOccupancy(obj)
            obj.grid = obj.baseGrid;
            keysList = obj.agvOccupancy.keys;
            for i = 1:numel(keysList)
                position = obj.agvOccupancy(keysList{i});
                obj.grid(position(1), position(2)) = 2;
            end
        end

        function entries = exportMapEntries(~, mapStore, fieldName)
            keysList = sort(cell2mat(mapStore.keys));
            entries = repmat(struct('id', [], fieldName, []), 0, 1);
            for i = 1:numel(keysList)
                entry = struct('id', keysList(i), fieldName, mapStore(keysList(i)));
                entries(end + 1, 1) = entry; %#ok<AGROW>
            end
        end

        function validateGrid(~, grid)
            validateattributes(grid, {'numeric'}, {'2d', 'nonempty', 'finite', '>=', 0, '<=', 4});
            if any(mod(grid(:), 1) ~= 0)
                error('MapClass:InvalidGrid', 'Grid values must be integers in the range 0..4.');
            end
        end
    end

    methods (Static)
        function obj = createDefaultMap()
            %CREATEDEFAULTMAP Build the default warehouse map with AGV spawns.
            obj = map.MapClass(map.MapClass.createDefaultGrid(), map.MapClass.defaultColors());
            positions = map.MapClass.defaultAGVPositions();
            for agvId = 1:size(positions, 1)
                obj.setAGVOccupancy(agvId, positions(agvId, 1), positions(agvId, 2));
            end
        end

        function grid = createDefaultGrid()
            %CREATEDEFAULTGRID Create the default static warehouse layout.
            mapRow = 28;
            mapCol = 62;
            grid = zeros(mapRow, mapCol);

            platformRows = 1:2;
            leftArea = 1:20;
            rightArea = 43:62;
            grid(platformRows, leftArea) = 3;
            grid(platformRows, rightArea) = 3;

            shelfRows = [5:6, 9:10, 13:14, 17:18, 21:22, 25:26];
            shelfCols = [3:10, 13:20, 23:30, 33:40, 43:50, 53:60];
            grid(shelfRows, shelfCols) = 1;
        end

        function positions = defaultAGVPositions()
            %DEFAULTAGVPOSITIONS Return the default AGV parking coordinates.
            positions = [1, 25; 1, 32; 1, 38];
        end

        function colors = defaultColors()
            %DEFAULTCOLORS Return the project color palette for cell values.
            colors = [
                1.0, 1.0, 1.0
                0.0, 0.8, 0.0
                1.0, 0.0, 0.0
                0.7, 0.7, 0.7
                0.0, 0.0, 1.0
            ];
        end
    end
end
