clear;
close all;

ReadFromData = true;
scene = 'stop';
debug_file_config = readDebugFile(ReadFromData, scene);

config = readConfig(debug_file_config('test_config_file_locate'));
   
plotDistanceMap(debug_file_config('distance_map_config_locate'), ...
                debug_file_config('distance_map_locate'));

[final_path, tree, result_vehicle, result_obstacle] = parseLog(debug_file_config('rrt_debug_locate'));

% figure 1
figure(1);
hold on;
plotComplexEnv;
axis equal;
plotReferencePath(config, debug_file_config('road_file_locate'));
plotMotion(config, result_vehicle, result_obstacle);

% figure 2
plotTree(tree);
plotPath(final_path);

% figure 6
figure(5);
hold on;
plotComplexEnv;
axis equal;
plotReferencePath(config, debug_file_config('road_file_locate'));
plotInitState(config, result_obstacle);
