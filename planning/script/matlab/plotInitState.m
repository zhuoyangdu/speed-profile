figure(4);
plotEnv;

plot(road(:,1), road(:,2), 'r');

plotCar(veh_x0, veh_y0, veh_theta0,'r', 1);

for i=1:1:1
    for k = 0:1:0
        ox = obs(i,1) + obs(i,4)*k*sin(obs(i,3));
        oy = obs(i,2) + obs(i,4)*k*cos(obs(i,3));
        plotCar(ox,oy,obs(i,3),'b',1-k*0.2);
    end
end
if length(obs(:,1))==2
    for i=2:1:2
        for k = 0:1:0
            ox = obs(i,1) + obs(i,4)*k*sin(obs(i,3));
            oy = obs(i,2) + obs(i,4)*k*cos(obs(i,3));
            plotCar(ox,oy,obs(i,3),'g',1-k*0.2);
        end
    end
end
for i = 1:1:1
    plotCar(result_vehicle(i,1), result_vehicle(i,2), result_vehicle(i,3), 'r', 1.2-i*0.2);
end

xlabel('x(m)');
ylabel('y(m)');