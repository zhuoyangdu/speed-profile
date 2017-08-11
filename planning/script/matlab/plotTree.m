function plotTree
log_file = fopen('../../log/rrt.txt');

while 1
    tline = fgetl(log_file);
    if ~ischar(tline), break, end
    if strcmp(tline, 'path')
        path = [];
        while 1
            tline = fgetl(log_file);
            if strcmp(tline, 'end_path'), break; end
            sline = str2double(strsplit(tline, '\t'));
            path = [path; sline];
        end
        plotPath(path);

    else if strcmp(tline, 'tree')
        tree = [];
        while 1
            tline = fgetl(log_file);
            if strcmp(tline, 'end_tree'), break; end
            sline = str2double(strsplit(tline, '\t'));
            tree = [tree; sline];
        end
        plot_tree(tree);
  
        else if contains(tline, 'sample')
            sline = strsplit(tline, '\t');
            sample_t = str2double(sline(2));
            sample_s = str2double(sline(3));
            figure(2);
            plot(sample_t, sample_s, '.');
            end
        end
    end
end
fclose(log_file);
end

function plotPath(path)
    figure(4);
    hold on;
    for i = 1:1:length(path(:,1))-1
        cn = path(i,:);
        pn = path(i+1,:);
        plot([cn(1), pn(1)], [cn(3), pn(3)], 'r', 'LineWidth',2);
        hold on;
    end  
end

function plot_tree(tree)
    figure(2);  
    hold on;
    for i = 2:1:length(tree(:,1))
        cn = tree(i,:);
        pn = tree(cn(4)+1,:);
        plot([cn(1), pn(1)], [cn(2), pn(2)], 'b', 'LineWidth',1);
        hold on;
    end
end