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
jsonText = jsonencode(agvPoolData, PrettyPrint=true);
fid = fopen(fullfile(outputDir, 'agv_pool.json'), 'w');
if fid < 0
    error('createAgvPool:OpenFailed', 'Failed to open agv_pool.json for writing.');
end
fprintf(fid, '%s', jsonText);
fclose(fid);

disp('Saved data/agv_pool.mat');
disp('Saved data/agv_pool.json');
end
