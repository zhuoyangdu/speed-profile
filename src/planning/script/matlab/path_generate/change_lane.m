width = 3.75;
% start and end condition
x0 = -160;
y0 = -5.625;
dx0 = 10;
dy0 = 0;
ddx0 = 0;
ddy0 = 0;

x1 = -110;
y1 = -1.875;
dx1 = 12;
ddx1 = 0;
dy1 = 0;
ddy1 = 0;

dt = 5;

%solve 
coef_x = solve_polynomial(0, dt, x0, x1, dx0, dx1, ddx0, ddx1);
coef_y = solve_polynomial(0, dt, y0, y1, dy0, dy1, ddy0, ddy1);

path_x = [];
path_y = [];

for t = 0:0.2:5
    path_x = [path_x; poly_value(coef_x, t, 0)];
    path_y = [path_y; poly_value(coef_y, t, 0)];    
end

before = linspace(-300, -165, 30)';
after = linspace(-95, -12, 15)';
path_x = [before; path_x; after];
path_y = [ones(size(before))*y0; path_y; ones(size(after))*y1];

%%% circle %%%
circle_x = [];
circle_y = [];

rx = -11.25;
ry = 11.25;
r = 3.75*3.5;
theta = 0;
while theta <= pi/2
    circle_x = [circle_x; rx + r * sin(theta)];
    circle_y = [circle_y; ry - r * cos(theta)];
    theta = theta + pi/20;
end

after = linspace(12, 500, 100)';
path_x = [path_x; circle_x; ones(size(after))*width/2];
path_y = [path_y; circle_y; after];

fid = fopen('change_lane_path.txt', 'w');
for i=1:1:length(path_x)
    fprintf(fid, "%.3f,%.3f\n", path_x(i),path_y(i));
end
fclose(fid);

figure(1);
hold on;
plot(path_x, path_y)
