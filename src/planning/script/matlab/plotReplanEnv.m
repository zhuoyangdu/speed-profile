figure(1);
hold on;
plotEnv;

plot(550:-1:460, 502.5*ones(1,91),'g');

plotCar(430, 497.5, pi/2,'k', 1);
plotCar(480, 502.5, -pi/2,'b', 1);
plotCar(550, 502.5, -pi/2, 'g', 1);
plotCar(502.5, 450, 0, 'r', 1);
xlim([400,560]);

plot(502.5*ones(1,46), 450:1:495, 'r--', 'LineWidth', 2);
plot(496:-1:400, 502.5*ones(1,97), 'r--', 'LineWidth', 2);
plot(495+7.5*cos(0:0.1:pi/2), 495+7.5*sin(0:0.1:pi/2),'r--', 'LineWidth', 2);

plot(480:-1:400, 502.5*ones(1,81), 'b');

plot(430:1:495, 497.5*ones(1,66), 'k');
plot(502.5*ones(1,67), 504:1:570,'k');
plot(495+7.5*sin(0:0.1:pi/2), 505-7.5*cos(0:0.1:pi/2), 'k');


