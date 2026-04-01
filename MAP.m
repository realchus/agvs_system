clear; clc; close all;

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);

warehouseMap = map.MapClass.createDefaultMap();
warehouseMap.plot();
