clear;
close all;

config = readConfig;
[final_path, tree, result_vehicle, result_obstacle] = parseLog;

% figure 1
plotMotion(config, result_vehicle, result_obstacle);

% figure 2
plotDistanceMap;
plotTree(tree);

%plotInitState;