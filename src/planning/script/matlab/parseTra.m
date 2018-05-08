log_file = fopen('../../log/trajectory_log');
path = [];
figure(6);
hold on;
while 1
    tline = fgetl(log_file);
    if ~ischar(tline), break, end
    if strcmp(tline, 'trajectory')
        if length(path)
            plot(path(:,1), path(:,2));
        end
        path = [];
    else
        sline = str2double(strsplit(tline,'\t'));
        path = [path;sline];
    end
end

a = load('../../log/vehicle_log');
tt = a(:,1);
vv = a(:,4);
plot(tt,vv, 'LineWidth',3);