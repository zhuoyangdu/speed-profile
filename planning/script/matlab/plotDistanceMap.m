A = load('../../log/distance_map.txt');
veh_s0 = A(1,1);
figure(2);
xlim([0,5]);
B = A(2:end,:);

[row,col] = size(B);
t = 0:0.1:t_goal;
s = (0:1:t_goal*max_vel) + veh_s0;
B = B(1:length(t), 1:length(s));
%t = (1:1:row)*0.1;
%s = veh_s0 + (1:1:col);
%pcolor(t, s, B');
%shading interp;
%hold on;
plot([0,t_goal],[veh_s0,t_goal*max_vel+veh_s0],'k--', 'LineWidth',2);

for i = 1:1:length(t)
    for j = 1:1:length(s)
        if B(i,j) < danger_distance
            plot(i*0.1, j + veh_s0, '*');
        end
        
    end
end


    