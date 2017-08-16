figure(1);
title('Environment');
hold on;
plot([495,495],[0,495], 'b');
plot([495,495],[505,1000], 'b');
plot([505,505],[0,495], 'b');
plot([505,505],[505,1000], 'b');
plot([0,495],[495,495], 'b');
plot([505,1000],[495,495], 'b');
plot([0,495],[505,505], 'b');
plot([505,1000],[505,505], 'b');
plot([500,500],[0,1000], 'g--');
plot([0,1000],[500,500], 'g--');
axis([400,600,400,600]);
plot(road(:,1), road(:,2), 'r');

plotCar(veh_x0, veh_y0, veh_theta0,'r', 1);

for i=1:1:length(obs(:,1))
    for k = 0:1:4
        ox = obs(i,1) + obs(i,4)*k*sin(obs(i,3));
        oy = obs(i,2) + obs(i,4)*k*cos(obs(i,3));
        plotCar(ox,oy,obs(i,3),'b',1-k*0.2);
    end
end

for i = 1:1:5
    plotCar(result_vehicle(i,1), result_vehicle(i,2), result_vehicle(i,3), 'r', 1.2-i*0.2);
end

axis([470,550,460,540]);
axis equal;
xlabel('x(m)');
ylabel('y(m)');