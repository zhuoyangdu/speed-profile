log_file = fopen('../../log/log_tree_17_08_08_19_54_23');
lines = [];
while 1
    tline = fgetl(log_file);
    % fprintf('%s, \n', tline);
    lines = [lines; string(tline)];
    if ~ischar(tline), break, end
end
fclose(log_file);

handle = [];
tree = [];
figure(2);
hold on;