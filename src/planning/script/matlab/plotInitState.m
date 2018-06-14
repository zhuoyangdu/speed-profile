function plotInitState(config, result_obstacle)


%%%% plot vehicle %%%
veh_x0 = str2double(config('veh_x0'));
veh_y0 = str2double(config('veh_y0'));
veh_theta0 = str2double(config('veh_theta0'));
plotCar(veh_x0, veh_y0, veh_theta0,'r', 1);

xlabel('x(m)');
ylabel('y(m)');
color_list = [0,1,0;
              0,0,1;
              1,1,0;
              1,1,1;
              0.5,0.5,0.5];
          
[obs_size, ~, ~] = size(result_obstacle);
for i = 1:1:obs_size
        plotCar(result_obstacle(i,1,1), result_obstacle(i,1,2), result_obstacle(i,1,3), color_list(i,:), 1);  
end
end