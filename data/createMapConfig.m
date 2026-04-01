function createMapConfig()
%CREATEMAPCONFIG Generate and save the default map configuration file.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

outputDir = fullfile(projectRoot, 'data');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

warehouseMap = map.MapClass.createDefaultMap();
mapConfig = warehouseMap.toStruct();

save(fullfile(outputDir, 'map_config.mat'), 'mapConfig');
disp('Saved data/map_config.mat');
end
