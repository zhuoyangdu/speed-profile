road = load('../../data/RoadXY.txt');
fid = fopen('../../config/planning.yaml');
yaml = textscan(fid, '%s%s%s', 'EndOfLine', '\n');
name = yaml{1,1};
value = yaml{1,3};

% import initial values
index = ismember(name,'single_test/x0');
veh_x0 = str2double(value{index});
index = ismember(name,'single_test/y0');
veh_y0 = str2double(value{index});
index = ismember(name,'single_test/theta0');
veh_theta0 = str2double(value{index});
index = ismember(name,'single_test/v0');
veh_v0 = str2double(value{index});
index = ismember(name,'single_test/obs1_x0');
obs1_x0 = str2double(value{index});
index = ismember(name,'single_test/obs1_y0');
obs1_y0 = str2double(value{index});
index = ismember(name,'single_test/obs1_theta0');
obs1_theta0 = str2double(value{index});
index = ismember(name,'single_test/obs1_v0');
obs1_v0 = str2double(value{index});
index = ismember(name,'single_test/obs2_x0');
obs2_x0 = str2double(value{index});
index = ismember(name,'single_test/obs2_y0');
obs2_y0 = str2double(value{index});
index = ismember(name,'single_test/obs2_theta0');
obs2_theta0 = str2double(value{index});
index = ismember(name,'single_test/obs2_v0');
obs2_v0 = str2double(value{index});

veh = [veh_x0, veh_y0, veh_theta0, veh_v0];
%obs = [obs1_x0,obs1_y0, obs1_theta0, obs1_v0; obs2_x0, obs2_y0, obs2_theta0, obs2_v0];
%obs = [obs1_x0,obs1_y0, obs1_theta0, obs1_v0];
obs = [obs2_x0,obs2_y0, obs2_theta0, obs2_v0];
% import config
index = ismember(name,'rrt/max_vel');
max_vel = str2double(value{index});
index = ismember(name,'rrt/t_goal');
t_goal = str2double(value{index});
max_dis = t_goal * max_vel;

index = ismember(name,'rrt/danger_distance');
danger_distance = str2double(value{index});

