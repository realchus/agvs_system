function agvPool = AGVPoolParser(sourcePath)
%AGVPOOLPARSER Load AGV pool definitions from a MAT or JSON file.
% Input:
%   sourcePath - Path to .mat or .json AGV pool data.
% Output:
%   agvPool    - Column vector of agv.AGVClass objects.

if nargin < 1 || isempty(sourcePath)
    sourcePath = fullfile(pwd, 'data', 'agv_pool.mat');
end

validateattributes(sourcePath, {'char', 'string'}, {'nonempty'});
sourcePath = char(string(sourcePath));

if ~isfile(sourcePath)
    error('AGVPoolParser:FileNotFound', 'AGV pool file not found: %s', sourcePath);
end

[~, ~, ext] = fileparts(sourcePath);
switch lower(ext)
    case '.mat'
        rawPool = loadFromMat(sourcePath);
    case '.json'
        rawPool = loadFromJson(sourcePath);
    otherwise
        error('AGVPoolParser:UnsupportedFormat', ...
            'Unsupported AGV pool file format: %s. Use .mat or .json.', ext);
end

agvPool = buildAgvPool(rawPool);
end

function rawPool = loadFromMat(sourcePath)
%LOADFROMMAT Read AGV payloads from a MAT file.
data = load(sourcePath);
candidateFields = {'agvPoolData', 'agvPool', 'agvs'};
for i = 1:numel(candidateFields)
    if isfield(data, candidateFields{i})
        rawPool = data.(candidateFields{i});
        return;
    end
end

error('AGVPoolParser:MissingPoolVariable', ...
    'MAT file %s does not contain agvPoolData, agvPool, or agvs.', sourcePath);
end

function rawPool = loadFromJson(sourcePath)
%LOADFROMJSON Read AGV payloads from a JSON file.
rawText = fileread(sourcePath);
rawPool = jsondecode(rawText);
end

function agvPool = buildAgvPool(rawPool)
%BUILDAGVPOOL Convert parsed structs into AGVClass objects.
if isempty(rawPool)
    agvPool = repmat(agv.AGVClass(1, [1, 1], 1.0), 0, 1);
    return;
end

if ~isstruct(rawPool)
    error('AGVPoolParser:InvalidPoolData', 'Parsed AGV pool data must be a struct array.');
end

agvPool = repmat(agv.AGVClass(1, [1, 1], 1.0), numel(rawPool), 1);
for i = 1:numel(rawPool)
    agvId = readField(rawPool(i), {'id'});
    position = readField(rawPool(i), {'position', 'start', 'startPos'});
    speed = readField(rawPool(i), {'speed'}, 1.0);

    agvObj = agv.AGVClass(agvId, position, speed);

    state = readField(rawPool(i), {'state'}, 'idle');
    agvObj.updateState(state);

    loadDuration = readField(rawPool(i), {'loadDuration', 'load_duration'}, agvObj.loadDuration);
    unloadDuration = readField(rawPool(i), {'unloadDuration', 'unload_duration'}, agvObj.unloadDuration);
    agvObj.loadDuration = double(loadDuration);
    agvObj.unloadDuration = double(unloadDuration);

    agvPool(i, 1) = agvObj;
end
end

function value = readField(dataStruct, names, defaultValue)
%READFIELD Read the first available field alias from data struct.
if nargin < 3
    defaultValue = [];
end

for i = 1:numel(names)
    if isfield(dataStruct, names{i}) && ~isempty(dataStruct.(names{i}))
        value = dataStruct.(names{i});
        return;
    end
end

if ~isempty(defaultValue)
    value = defaultValue;
    return;
end

error('AGVPoolParser:MissingField', 'AGV data is missing required field: %s', names{1});
end
