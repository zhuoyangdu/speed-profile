function plotCar(x,y,ang, color)
    figure(1);
    hold on;

    length = 6;
    width = 3;
 
    pos_f = [x + length/2 * sin(ang), y + length/2 * cos(ang)];
    pos_r = [x - length/2 * sin(ang), y - length/2 * cos(ang)];
    %pos_fl = [pos_f(1) - width/2 * sin(ang), pos_f(2) + width/2 * cos(ang)];
    %pos_fr = [pos_f(1) + width/2 * sin(ang), pos_f(2) - width/2 * cos(ang)];
    pos_rl = [pos_r(1) - width/2 * cos(ang), pos_r(2) + width/2 * sin(ang)];
    pos_rr = [pos_r(1) + width/2 * cos(ang), pos_r(2) - width/2 * sin(ang)];
    plot([pos_f(1), pos_rr(1), pos_rl(1), pos_f(1)], [pos_f(2), pos_rr(2), pos_rl(2), pos_f(2)],color);
    fill([pos_f(1), pos_rr(1), pos_rl(1), pos_f(1)], [pos_f(2), pos_rr(2), pos_rl(2), pos_f(2)],color);
    
end