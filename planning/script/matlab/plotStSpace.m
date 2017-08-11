readConfig;
% path spline
x = road(:,1);
y = road(:,2);
path_s = sqrt((x(2:end) - x(1:end-1)).^2 + (y(2:end) - y(1:end-1)).^2);
path_s = [0; cumsum(path_s)];
path_x = spline(path_s,x);
path_y = spline(path_s,y);

% compute initial path length of vehicle
distance = (x - veh(1)).^2 + (y - veh(2)).^2;
[sort_dis, indexes] = sort(distance);
closest1 = road(indexes(1),:);
s1 = path_s(indexes(1));
closest2 = road(indexes(2),:);
s2 = path_s(indexes(2));
k = sqrt((veh(1)-closest1(1))^2 + (veh(2)-closest1(2))^2)/sqrt((veh(1)-closest2(1))^2 + (veh(2)-closest2(2))^2);
veh_s0 = (k*s2+s1)/(k+1);
veh_splinex = ppval(path_x, veh_s0);
veh_spliney = ppval(path_y, veh_s0);

n_obs = length(obs(:,1));
space_t = linspace(0, t_max, 100)';
space_s = linspace(0, s_max, 100)';
collision_map = ones(length(space_t), length(space_s));
distance_map = ones(length(space_t), length(space_s))*1000;

for i = 1:1:length(space_t)
    for j = 1:1:length(space_s)
        t = space_t(i);
        s = space_s(j);
        veh_x = ppval(path_x, s+veh_s0);
        veh_y = ppval(path_y, s+veh_s0);
        for k = 1:1:n_obs
            obs_x = obs(k,1) + obs(k,4)*t*sin(obs(k,3));
            obs_y = obs(k,2) + obs(k,4)*t*cos(obs(k,3));
            d = sqrt((veh_x-obs_x)^2 + (veh_y - obs_y)^2);
            if distance_map(i,j)>d
                distance_map(i,j) = d;
            end
            if d < danger_distance
                collision_map(i,j) = 0;
            end
        end
    end
end

figure(2);
title('s-t motion space');
pcolor(space_t, space_s+veh_s0, distance_map');
shading interp;
hold on;
for i = 1:1:length(space_t)
    for j = 1:1:length(space_s)
        if collision_map(i,j) == 0
            plot(space_t(i), space_s(j)+veh_s0,'x');
        end
    end
end
plot([0,t_max],[veh_s0,t_max*max_vel+veh_s0],'-','LineWidth',3);