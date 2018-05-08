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
    %postProcessing;
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
    %plotTree(tree);
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
end
fclose(log_file);
