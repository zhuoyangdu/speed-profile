width = 3.75;
 
path_x = linspace(-500, -12, 100)';
path_y = -ones(size(path_x))*5.78;

%%% circle %%%
circle_x = [];
circle_y = [];

rx = -11.25;
ry = 11.25;
r = 17.03;
theta = 0;
while theta <= pi/2
    circle_x = [circle_x; rx + r * sin(theta)];
    circle_y = [circle_y; ry - r * cos(theta)];
    theta = theta + pi/20;
end

after = linspace(12, 500, 100)';
path_x = [path_x; circle_x; ones(size(after))*5.78];
path_y = [path_y; circle_y; after];

fid = fopen('center_lane.txt', 'w');
for i=1:1:length(path_x)
    fprintf(fid, "%.3f,%.3f\n", path_x(i),path_y(i));
end
fclose(fid);

figure(1);
hold on;
plot(path_x, path_y)
