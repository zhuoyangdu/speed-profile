function plotMotion(config, result_vehicle, result_obstacle)
figure(1);
plotComplexEnv;
axis equal;

% plot reference path.
road_file = ['../../../simulation/data/path/',config('road_file')];
fid = fopen(road_file);
yaml = textscan(fid, '%f,%f', 'EndOfLine', '\n');
path_x = yaml{1,1};
path_y = yaml{1,2};
fclose(fid);

% plot map
plot(path_x, path_y);


%%%% plot vehicle %%%
veh_x0 = str2double(config('veh_x0'));
veh_y0 = str2double(config('veh_y0'));
veh_theta0 = str2double(config('veh_theta0'));
plotCar(veh_x0, veh_y0, veh_theta0,'r', 1);

for i = 1:1:5
    plotCar(result_vehicle(i,1), result_vehicle(i,2), result_vehicle(i,3), 'r', 1.2-i*0.2);
end

xlabel('x(m)');
ylabel('y(m)');

[obs_size, ~, ~] = size(result_obstacle);
for i = 1:1:obs_size
    for k = 1:1:5
        plotCar(result_obstacle(i,k,1), result_obstacle(i,k,2), result_obstacle(i,k,3), 'g', 1.2-k*0.2);
    end   
end
