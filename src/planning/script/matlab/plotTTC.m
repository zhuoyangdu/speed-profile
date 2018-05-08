log_file = fopen('../../log/ttc_map.txt');
ttc_map = [];
i = 1;
while 1
    tline = fgetl(log_file);
    if ~ischar(tline), break, end
    if contains(tline, 'velocity')
    else
        sline=str2double(strsplit(tline, '\t'));
        ttc_map = [ttc_map; sline(1:end-1)];
    end
end

for i = 0:2:8
    ttc_map_v = ttc_map(80*i+1:80*i+80,:,:);
    figure;
    title(i);
    t = 0:0.1:(t_max-0.1);
    s = 0:1:(s_max-1);
    pcolor(t, s, ttc_map_v');
    shading interp;
end