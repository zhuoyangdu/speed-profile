function [final_path, tree, result_vehicle, result_obstacle] = parseLog

log_file = fopen('../../log/rrt.txt');
final_path = [];
while 1
tline = fgetl(log_file);
if ~ischar(tline), break, end
if strcmp(tline, 'final_path')
    path = [];
    while 1
        tline = fgetl(log_file);
        if strcmp(tline, 'end_path'), break; end
        sline = str2double(strsplit(tline, '\t'));
        path = [path; sline];
    end
    final_path = path;
    %plotPath(path);
end
if strcmp(tline, 'smoothing_path')
    path = [];
    while 1
        tline = fgetl(log_file);
        if strcmp(tline, 'end_path'), break; end
        sline = str2double(strsplit(tline, '\t'));
        path = [path; sline];
    end
    smooth_path = path;
    %plotPath(path); 
end
if strcmp(tline, 'tree')
    tree = [];
    while 1
        tline = fgetl(log_file);
        if strcmp(tline, 'end_tree'), break; end
        sline = str2double(strsplit(tline, '\t'));
        tree = [tree; sline];
    end
end
if strcmp(tline, 'moving_vehicle')
    result_vehicle = [];
    while 1
        tline = fgetl(log_file);
        if strcmp(tline, 'end_vehicle'), break; end
        sline = str2double(strsplit(tline, '\t'));
        result_vehicle = [result_vehicle; sline];
    end
end

if strcmp(tline, 'moving_obstacle')
    tline = fgetl(log_file);
    sline = strsplit(tline, '\t');
    obs_size = str2double(sline{1,2});
    [row,col] = size(result_vehicle);
    result_obstacle = zeros(obs_size, row, col);
    num_obs = 1;
    while 1
        tline = fgetl(log_file);
        if strcmp(tline, 'end_obstacle'), break; end
        if strcmp(tline, 'start') obstacle = []; continue; end
        if strcmp(tline, 'end') 
            result_obstacle(num_obs, :, :) = obstacle; 
            num_obs=num_obs+1; 
            continue; 
        end  
        sline = str2double(strsplit(tline, '\t'));
        obstacle = [obstacle; sline];
    end
end

end
fclose(log_file);
end
