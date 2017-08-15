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
    plotPath(path);

else if strcmp(tline, 'tree')
    tree = [];
    while 1
        tline = fgetl(log_file);
        if strcmp(tline, 'end_tree'), break; end
        sline = str2double(strsplit(tline, '\t'));
        tree = [tree; sline];
    end
    plotTree(tree);
    %{
    else if contains(tline, 'sample')
        sline = strsplit(tline, '\t');
        sample_t = str2double(sline(2));
        sample_s = str2double(sline(3));
        figure(2);
        plot(sample_t, sample_s, '.');
        end
    %}
    end
end
end
fclose(log_file);
