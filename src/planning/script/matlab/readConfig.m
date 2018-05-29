function config = readConfig()
%%%%%%%%%%% read planning config %%%%%%%%%%%%%%%%%
fid = fopen('../../config/planning_config.pb.txt');
[name, value] = parseProto('rrt {', fid);

%%%%%%%%%%% read init state config %%%%%%%%%%%%%%%%%
fid = fopen('../../config/junction_test_config.pb.txt');

[vehicle_name, vehicle_value] = parseProto('init_vehicle_state {', fid);
for i = 1:1:length(vehicle_name)
    vehicle_name{i,1} = ['veh_', vehicle_name{i,1}];
end

name = [name; vehicle_name];
value = [value; vehicle_value];

config = containers.Map(name, value);
end

function [name, value] = parseProto(message, fid)
    tline = fgetl(fid);
    while ischar(tline)
        % disp(tline);
        if strcmp(tline, message)
            disp('break');
            break;            
        end
        tline = fgetl(fid);
    end

    name = [];
    value = [];
    tline = fgetl(fid);
    while ischar(tline) 
        % disp(tline);
        if strcmp(tline, '')
            continue;
        end
        if strcmp(tline, '}')
            break;
        end
        tline = regexprep(tline, ' ','');
        tline = regexprep(tline, '\t','');
        split_line = split(tline, ':');
        name = [name; {char(split_line(1))}];
        value = [value; {char(split_line(2))}];
        tline = fgetl(fid);
    end

end
