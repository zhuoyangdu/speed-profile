clear;
close all;
readConfig;
parseLog;
plotDistanceMap;
%plotPath(smooth_path);
plotTree(tree);
plotPath(final_path);
plotMotion;
plotInitState;

figure(1);
set(gcf, 'position', [0 0 300 200]);
figure(2);
set(gcf, 'position', [0 0 300 200]);
figure(3);
set(gcf, 'position', [0 0 300 200]);
figure(4);
set(gcf, 'position', [0 0 300 200]);