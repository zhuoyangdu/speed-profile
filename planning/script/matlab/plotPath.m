function plotPath(path)
figure(4);
title('path');
hold on;
for i = 1:1:length(path(:,1))-1
    cn = path(i,:);
    pn = path(i+1,:);
    plot([cn(1), pn(1)], [cn(3), pn(3)], 'r', 'LineWidth',2);
    hold on;
end  
figure(2);
hold on;
for i = 1:1:length(path(:,1))-1
    cn = path(i,:);
    pn = path(i+1,:);
    plot([cn(1), pn(1)], [cn(2), pn(2)], 'r', 'LineWidth',2);
    hold on;
end     

