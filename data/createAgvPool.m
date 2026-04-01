function createAgvPool()
%CREATEAGVPOOL Generate and save the default AGV pool data file.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);

outputDir = fullfile(projectRoot, 'data');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

agvPool = agv.AGVClass.createDefaultPool();
agvPoolData = agv.AGVClass.poolToStructArray(agvPool);

save(fullfile(outputDir, 'agv_pool.mat'), 'agvPoolData');
disp('Saved data/agv_pool.mat');
end
