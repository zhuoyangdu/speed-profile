% road = load('../../data/RoadXY.txt');
fid = fopen('../../config/planning.yaml');
yaml = textscan(fid, '%s%s%s', 'EndOfLine', '\n');
name = yaml{1,1};
value = yaml{1,3};

% import initial values
index = ismember(name,'single_test/single_test_case');
single_case = value{index};
single_case = single_case(2:end-1);

index = ismember(name,'single_test/collision_number');
collision_number = str2double(value{index});

index = ismember(name,[single_case,'/road_file']);
road_path = ['../../data/path/', value{index}(2:end-1)];
road = load(road_path);
    
index = ismember(name,[single_case,'/x0']);
veh_x0 = str2double(value{index});
index = ismember(name,[single_case,'/y0']);
veh_y0 = str2double(value{index});
index = ismember(name,[single_case,'/theta0']);
veh_theta0 = str2double(value{index});
index = ismember(name,[single_case,'/v0']);
veh_v0 = str2double(value{index});
if collision_number > 0
    index = ismember(name,[single_case,'/obs1_x0']);
    obs1_x0 = str2double(value{index});
    index = ismember(name,[single_case,'/obs1_y0']);
    obs1_y0 = str2double(value{index});
    index = ismember(name,[single_case,'/obs1_theta0']);
    obs1_theta0 = str2double(value{index});
    index = ismember(name,[single_case,'/obs1_v0']);
    obs1_v0 = str2double(value{index});
end
if collision_number > 1
    index = ismember(name,[single_case,'/obs2_x0']);
    obs2_x0 = str2double(value{index});
    index = ismember(name,[single_case,'/obs2_y0']);
    obs2_y0 = str2double(value{index});
    index = ismember(name,[single_case,'/obs2_theta0']);
    obs2_theta0 = str2double(value{index});
    index = ismember(name,[single_case,'/obs2_v0']);
    obs2_v0 = str2double(value{index});
end

veh = [veh_x0, veh_y0, veh_theta0, veh_v0];
obs = [];
if collision_number == 1
    obs = [obs1_x0,obs1_y0, obs1_theta0, obs1_v0];
end
if collision_number == 2
    obs = [obs1_x0,obs1_y0, obs1_theta0, obs1_v0; obs2_x0, obs2_y0, obs2_theta0, obs2_v0];  
end

% import config
index = ismember(name,'rrt/max_vel');
max_vel = str2double(value{index});
index = ismember(name,'rrt/t_goal');
t_goal = str2double(value{index});
max_dis = t_goal * max_vel;
index = ismember(name,'rrt/t_max');
t_max = str2double(value{index});
index = ismember(name,'rrt/s_max');
s_max = str2double(value{index});

index = ismember(name,'rrt/danger_distance');
danger_distance = str2double(value{index});



