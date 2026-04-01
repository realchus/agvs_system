classdef Visualizer < handle
    %VISUALIZER Render the warehouse map, AGVs, paths, and obstacles.
    %   This helper keeps plotting code out of the simulation loop and can
    %   be disabled for headless automated tests.

    properties
        enabled
        figureHandle
        axesHandle
        visible
    end

    methods
        function obj = Visualizer(enabled, visible)
            %VISUALIZER Construct a visualizer instance.
            % Inputs:
            %   enabled - True to create/render figures.
            %   visible - Optional figure visibility flag.
            if nargin < 1 || isempty(enabled)
                enabled = true;
            end
            if nargin < 2 || isempty(visible)
                visible = enabled;
            end

            obj.enabled = logical(enabled);
            obj.visible = logical(visible);
            obj.figureHandle = [];
            obj.axesHandle = [];
        end

        function render(obj, mapObj, agvPool, currentTime, dynamicObstacles)
            %RENDER Draw the current simulation state.
            % Inputs:
            %   mapObj           - map.MapClass instance.
            %   agvPool          - AGV object array.
            %   currentTime      - Simulation time.
            %   dynamicObstacles - N-by-2 obstacle positions.
            if ~obj.enabled
                return;
            end

            if isempty(obj.figureHandle) || ~isgraphics(obj.figureHandle)
                visibility = 'off';
                if obj.visible
                    visibility = 'on';
                end
                obj.figureHandle = figure( ...
                    'Color', 'w', ...
                    'Name', 'Multi-AGV Simulation', ...
                    'NumberTitle', 'off', ...
                    'Visible', visibility);
                obj.axesHandle = axes('Parent', obj.figureHandle);
            end

            cla(obj.axesHandle);
            mapObj.plot(obj.axesHandle);
            hold(obj.axesHandle, 'on');

            for i = 1:numel(agvPool)
                agvObj = agvPool(i);
                if ~isempty(agvObj.path)
                    plot(obj.axesHandle, agvObj.path(:, 2), agvObj.path(:, 1), '--', ...
                        'LineWidth', 1.2, 'Color', [0.1, 0.4, 0.9]);
                end

                scatter(obj.axesHandle, agvObj.position(2), agvObj.position(1), 80, ...
                    'filled', 'MarkerFaceColor', [0.85, 0.1, 0.1], ...
                    'MarkerEdgeColor', [0.2, 0.2, 0.2]);
                text(obj.axesHandle, agvObj.position(2) + 0.2, agvObj.position(1), ...
                    sprintf('AGV%d (%s)', agvObj.id, agvObj.state), ...
                    'Color', [0.1, 0.1, 0.1], 'FontSize', 9, 'FontWeight', 'bold');
            end

            if nargin >= 5 && ~isempty(dynamicObstacles)
                scatter(obj.axesHandle, dynamicObstacles(:, 2), dynamicObstacles(:, 1), ...
                    90, 's', 'filled', 'MarkerFaceColor', [0.1, 0.1, 0.1], ...
                    'MarkerEdgeColor', [1.0, 0.85, 0.0]);
            end

            title(obj.axesHandle, sprintf('Simulation Time: %.1f s', currentTime));
            hold(obj.axesHandle, 'off');
            drawnow limitrate;
        end
    end
end
