function plotTree(tree)
    figure(2);  
    hold on;
    for i = 2:1:length(tree(:,1))
        cn = tree(i,:);
        pn = tree(cn(4)+1,:);
        plot([cn(1), pn(1)], [cn(2), pn(2)], 'b', 'LineWidth',1);
        hold on;
    end
end