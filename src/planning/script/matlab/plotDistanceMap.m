function plotDistanceMap(map_config_locate, map_locate)

C = fopen(map_config_locate);
yaml = textscan(C, '%s%f', 'EndOfLine', '\n');
name = yaml{1,1};
for i = 1:1:length(name) 
    name{i,1} = name{i,1}(1:end-1); 
end
value = yaml{1,2};
cfg = containers.Map(name, value);

A = load(map_locate);
figure(2);
xlim([0,5]);
ylim([cfg('init_path_length'), 5*cfg('max_vel')+cfg('init_path_length')]);
[row,col] = size(A);
t = 0:0.1:cfg('t_goal');
s = (0:1:cfg('t_goal')*cfg('max_vel')) + cfg('init_path_length');
A = A(1:length(t), 1:length(s));

hold on;
plot([0,cfg('t_goal')],[cfg('init_path_length') , ...
    cfg('t_goal')*cfg('max_vel')+cfg('init_path_length')],...
    'k--', 'LineWidth',2);

for i = 1:1:length(t)
    for j = 1:1:length(s)
        if A(i,j) < cfg('danger_distance')
            figure(2);
            plot(i*0.1, j + cfg('init_path_length'),'.','Color',[0.7,0.7,0.7],'MarkerSize',5);
        end
        
    end
end

end
    