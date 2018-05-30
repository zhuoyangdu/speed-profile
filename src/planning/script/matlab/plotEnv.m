function plotEnv(config)
% plot reference path.
road_file = ['../../../simulation/data/path/',config('road_file')];

fid = fopen(road_file);

yaml = textscan(fid, '%f,%f', 'EndOfLine', '\n');
path_x = yaml{1,1};
path_y = yaml{1,2};

% plot map
plot(path_x, path_y);
xlabel('x(m)');
ylabel('y(m)');

if strcmp(config('scene'),'complex')
    figure(1);
    hold on;
    axis equal;

    plot([11.25, 11.25], [11.25, 500], 'b');
    plot([-11.25, -11.25], [11.25, 500], 'b');
    plot([11.25, 11.25], [-11.25, -500], 'b');
    plot([-11.25, -11.25], [-11.25, -500], 'b');
    plot([0, 0], [11.25, 500], 'b');
    plot([0,0 ], [-11.25, -500], 'b');
    
    plot([11.25, 500], [11.25, 11.25], 'b');
    plot([11.25, 500], [-11.25, -11.25], 'b');
    plot([-11.25, -500], [11.25, 11.25], 'b');
    plot([-11.25, -500], [-11.25, -11.25], 'b');
    plot([11.25, 500], [0, 0], 'b');
    plot([-11.25, -500], [0, 0],  'b');
else 
    
scene = ['../../../simulation/data/', config('scene'), '.net.xml'];
xDoc= xmlread(scene);
xRoot = xDoc.getDocumentElement;
xnet= xRoot.getElementsByTagName('edge');
numEdge = xnet.getLength;
allEdges = cell(1, numEdge);
allEdgesShape = [];

for i = 1:1:numEdge
    if ~xnet.item(i-1).hasAttribute('function')
    allEdges{1,i} = xnet.item(i-1);
    numLanes = xnet.item(i-1).getElementsByTagName('lane').getLength;
    for k = 1:1:numLanes
        shape = xnet.item(i-1).getElementsByTagName('lane').item(k-1).getAttribute('shape');
        allEdgesShape = [allEdgesShape; {char(shape)}];
    end
    end
end

figure(1);
hold on;
for i = 1:1:length(allEdgesShape)
    edge = allEdgesShape{i,1};
    sp = regexp(edge, ' ', 'split');
    points = zeros(length(sp), 2);
    if length(sp) == 2
        disp(edge)
        for j = 1:1:length(sp)
            points(j,:) = double(strsplit(string(sp(j)), ','));
        end
        %plot(points(:,1), points(:,2));
        % disp(sp);
        for j = 1:1:length(sp)-1
            out = plotWidth(points(j,:), points(j+1,:), 3.75);
            plot([out(1,1), out(2,1)], [out(1,2), out(2,2)], 'b');
            plot([out(3,1), out(4,1)], [out(3,2), out(4,2)], 'b');
    end
    end
end

end
end

function outsider = plotWidth(point1, point2, width)
    x1 = point1(1);
    y1 = point1(2);
    x2 = point2(1);
    y2 = point2(2);
    theta = atan2(y2-y1,x2-x1);
    
    x11 = x1 - 0.5 * width * sin(theta);
    y11 = y1 + 0.5 * width * cos(theta);
    x12 = x1 + 0.5 * width * sin(theta);
    y12 = y1 - 0.5 * width * cos(theta);
 
    x21 = x2 - 0.5 * width * sin(theta);
    y21 = y2 + 0.5 * width * cos(theta);
    x22 = x2 + 0.5 * width * sin(theta);
    y22 = y2 - 0.5 * width * cos(theta);
    
    outsider = [x11, y11; x21, y21; x12, y12; x22, y22];
end





