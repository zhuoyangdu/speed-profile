A = load('../../log/distance_map.txt');
veh_s0 = A(1,1);
figure(2);
xlim([0,5]);
B = A(2:end,:);

[row,col] = size(B);
t = 0:0.1:t_goal;
s = (0:1:t_goal*max_vel) + veh_s0;
B = B(1:length(t), 1:length(s));

hold on;
plot([0,t_goal],[veh_s0,t_goal*max_vel+veh_s0],'k--', 'LineWidth',2);

for i = 1:1:length(t)
    for j = 1:1:length(s)
        if B(i,j) < danger_distance
            figure(2);
            plot(i*0.1, j + veh_s0, 'g.');
            %fprintf('%f,%f\n',i,j);
        end
        
    end
end


    