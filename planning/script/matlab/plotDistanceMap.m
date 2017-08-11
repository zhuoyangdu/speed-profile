A = load('../../log/distance_map.txt');
figure(2);
[row,col] = size(A);
t = (1:1:row)*0.1;
s = veh_s0 + (1:1:col);
pcolor(t, s, A');
shading interp;
hold on;
