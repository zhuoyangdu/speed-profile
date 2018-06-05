function plotReferencePath(config)
%PLOTREFERENCEPATH 此处显示有关此函数的摘要
%   此处显示详细说明

% plot reference path.
road_file = ['../../../simulation/data/path/',config('road_file')];
fid = fopen(road_file);
yaml = textscan(fid, '%f,%f', 'EndOfLine', '\n');
path_x = yaml{1,1};
path_y = yaml{1,2};
fclose(fid);

x0 = str2double(config('veh_x0'));
y0 = str2double(config('veh_y0'));

dis = (path_x-x0).^2 + (path_y-y0).^2;
[~, index] = min(dis);
start_x = path_x(index); start_y = path_y(index);
sum = 0;
for i = index:1:length(path_x)
    sum = sum + sqrt((path_x(i+1)-path_x(i))^2 + (path_y(i+1)-path_y(i))^2);
    if sum >= 80
        break;
    end
end
path_x = path_x(index:i);
path_y = path_y(index:i);
% plot map
plot(path_x, path_y);
end

