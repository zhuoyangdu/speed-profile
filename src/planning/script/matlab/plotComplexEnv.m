function plotComplexEnv
    figure(1);
    hold on;
    xlabel('x(m)');
    ylabel('y(m)');
    xlim([-500,500])
    ylim([-500,500])
    axis equal;

    % plot road
    x = [-500, -500, 500, 500];
    y = [-11.25, 11.25, 11.25, -11.25];
    fill(x,y,[.5,.5,.5],'linestyle','none');
    fill(y,x,[.5,.5,.5],'linestyle','none');

    % plot center line
    plot_lane(0,{'y',2});

    % plot lane
    plot_lane(3.75,{'w--',1});
    plot_lane(-3.75,{'w--',1});
    plot_lane(7.5,{'w--',1});
    plot_lane(-7.5,{'w--',1});
    
    %plot arrow
    plot_arrow(-14.5, -5.625,0, 'straight');
    plot_arrow(14.5, 5.625,pi, 'straight');
    plot_arrow(-5.625, 14.5, -pi/2, 'straight');
    plot_arrow(5.625, -14.5, pi/2, 'straight');
    
    plot_arrow(-14.5, -1.875,0, 'left');
    plot_arrow(14.5, 1.875,pi, 'left');
    plot_arrow(-1.875, 14.5, -pi/2, 'left');
    plot_arrow(1.875, -14.5, pi/2, 'left');  
    
    plot_arrow(-14.5, -9.375,0, 'right');
    plot_arrow(14.5, 9.375,pi, 'right');
    plot_arrow(-9.375, 14.5, -pi/2, 'right');
    plot_arrow(9.375, -14.5, pi/2, 'right');
    
end

function plot_lane(s, style)
    color = style{1,1};
    linewidth = style{1,2};
    plot([-500,-11.25], [s,s], color, 'LineWidth', linewidth);
    plot([500,11.25], [s,s], color, 'LineWidth', linewidth);
    plot([s,s],[-500,-11.25],color, 'LineWidth', linewidth);
    plot([s,s],[500,11.25], color, 'LineWidth', linewidth);
end

function plot_arrow(x,y,theta,type)
    if strcmp(type, 'straight')
        s1 = 0.5;
        s2 = 2;
        x1 = x - s1 * sin(theta);
        x2 = x + s1 * sin(theta);
        x3 = x + s2 * cos(theta);
        y1 = y + s1 * cos(theta);
        y2 = y - s1 * cos(theta);
        y3 = y + s2 * sin(theta);
        fill([x1,x2,x3],[y1,y2,y3], [1,1,1], 'linestyle','none');

        s1 = 0.15;
        s2 = 3;
        x1 = x - s1 * sin(theta);
        x2 = x + s1 * sin(theta);
        y1 = y + s1 * cos(theta);
        y2 = y - s1 * cos(theta);

        xx = x - s2 * cos(theta);
        yy = y - s2 * sin(theta);
        x4 = xx - s1 * sin(theta);
        x3 = xx + s1 * sin(theta);
        y4 = yy + s1 * cos(theta);
        y3 = yy - s1 * cos(theta);
        fill([x1,x2,x3, x4],[y1,y2,y3, y4], [1,1,1], 'linestyle','none');
    end

    if strcmp(type, 'left')

        s1 = 0.15;
        s2 = 3;
        x1 = x - s1 * sin(theta);
        x2 = x + s1 * sin(theta);
        y1 = y + s1 * cos(theta);
        y2 = y - s1 * cos(theta);

        xx = x - s2 * cos(theta);
        yy = y - s2 * sin(theta);
        x4 = xx - s1 * sin(theta);
        x3 = xx + s1 * sin(theta);
        y4 = yy + s1 * cos(theta);
        y3 = yy - s1 * cos(theta);
        fill([x1,x2,x3, x4],[y1,y2,y3, y4], [1,1,1], 'linestyle','none');
        
        s2 = -0.5;
        theta = theta + pi/4;
        x1 = x - s1 * sin(theta);
        x2 = x + s1 * sin(theta);
        y1 = y + s1 * cos(theta);
        y2 = y - s1 * cos(theta);

        xx = x - s2 * cos(theta);
        yy = y - s2 * sin(theta);
        x4 = xx - s1 * sin(theta);
        x3 = xx + s1 * sin(theta);
        y4 = yy + s1 * cos(theta);
        y3 = yy - s1 * cos(theta);
        fill([x1,x2,x3, x4],[y1,y2,y3, y4], [1,1,1], 'linestyle','none');
        
        x = xx;
        y = yy;
        
        s1 = 0.5;
        s2 = 1.5;
        x1 = x - s1 * sin(theta);
        x2 = x + s1 * sin(theta);
        x3 = x + s2 * cos(theta);
        y1 = y + s1 * cos(theta);
        y2 = y - s1 * cos(theta);
        y3 = y + s2 * sin(theta);
        fill([x1,x2,x3],[y1,y2,y3], [1,1,1], 'linestyle','none');
    end

    if strcmp(type, 'right')
        s1 = 0.15;
        s2 = 3;
        x1 = x - s1 * sin(theta);
        x2 = x + s1 * sin(theta);
        y1 = y + s1 * cos(theta);
        y2 = y - s1 * cos(theta);

        xx = x - s2 * cos(theta);
        yy = y - s2 * sin(theta);
        x4 = xx - s1 * sin(theta);
        x3 = xx + s1 * sin(theta);
        y4 = yy + s1 * cos(theta);
        y3 = yy - s1 * cos(theta);
        fill([x1,x2,x3, x4],[y1,y2,y3, y4], [1,1,1], 'linestyle','none');
        
        s2 = -0.5;
        theta = theta - pi/4;
        x1 = x - s1 * sin(theta);
        x2 = x + s1 * sin(theta);
        y1 = y + s1 * cos(theta);
        y2 = y - s1 * cos(theta);

        xx = x - s2 * cos(theta);
        yy = y - s2 * sin(theta);
        x4 = xx - s1 * sin(theta);
        x3 = xx + s1 * sin(theta);
        y4 = yy + s1 * cos(theta);
        y3 = yy - s1 * cos(theta);
        fill([x1,x2,x3, x4],[y1,y2,y3, y4], [1,1,1], 'linestyle','none');
        
        x = xx;
        y = yy;
        
        s1 = 0.5;
        s2 = 1.5;
        x1 = x - s1 * sin(theta);
        x2 = x + s1 * sin(theta);
        x3 = x + s2 * cos(theta);
        y1 = y + s1 * cos(theta);
        y2 = y - s1 * cos(theta);
        y3 = y + s2 * sin(theta);
        fill([x1,x2,x3],[y1,y2,y3], [1,1,1], 'linestyle','none');

    end    
end