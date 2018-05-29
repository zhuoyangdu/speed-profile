function plotEnv
% plot reference path.
fid = fopen('../../../simulation/data/path/junction_path.txt');
yaml = textscan(fid, '%f,%f', 'EndOfLine', '\n');
path_x = yaml{1,1};
path_y = yaml{1,2};

% plot map
plot(path_x, path_y);
xlabel('x(m)');
ylabel('y(m)');

xDoc= xmlread('../../../simulation/data/junction.net.xml');
xRoot = xDoc.getDocumentElement;
xnet= xRoot.getElementsByTagName('edge');
numEdge = xnet.getLength;
allEdges = cell(1, numEdge);
allEdgesShape = cell(1, numEdge);

for i = 1:1:numEdge
    allEdges{1,i} = xnet.item(i-1);
    shape = xnet.item(i-1).getElementsByTagName('lane').item(0).getAttribute('shape');
    allEdgesShape{1,i} = char(shape);
end

figure(1);
hold on;
for i = 1:1:numEdge
    edge = allEdgesShape{1,i};
    sp = regexp(edge, ' ', 'split');
    points = zeros(length(sp), 2);
    if length(sp) == 2
        for j = 1:1:length(sp)
            points(j,:) = double(strsplit(string(sp(j)), ','));
        end
        %plot(points(:,1), points(:,2));
        % disp(sp);
        for j = 1:1:length(sp)-1
            out = plotWidth(points(j,:), points(j+1,:), 5);
            plot([out(1,1), out(2,1)], [out(1,2), out(2,2)], 'b');
            plot([out(3,1), out(4,1)], [out(3,2), out(4,2)], 'b');
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





