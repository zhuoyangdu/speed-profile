function plotMotion(config, result_vehicle, result_obstacle)
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
color_list = [0,1,0;
              0,0,1;
              1,1,0;
              1,1,1;
              0.5,0.5,0.5];
for i = 1:1:obs_size
    for k = 1:1:5
        plotCar(result_obstacle(i,k,1), result_obstacle(i,k,2), result_obstacle(i,k,3), color_list(i,:), 1.2-k*0.2);
    end   
end
