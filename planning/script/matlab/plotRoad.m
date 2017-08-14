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

plotCar(veh_x0, veh_y0, veh_theta0,'r');
for i=1:1:length(obs(:,1))
    plotCar(obs(i,1), obs(i,2), obs(i,3), 'b');
end
