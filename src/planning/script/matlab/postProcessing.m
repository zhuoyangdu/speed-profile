path_t = [0:0.1:4.7]';
path_s = [];
for t = 0:0.1:4.7
    [dt, index] = sort(abs(final_path(:,1)-t));
    if dt(1) == 0
        path_s = [path_s;final_path(index(1),2)];
    else
        index1 = 0;
        index2 = 0;
        if t > final_path(index(1),1)
            index1 = index(1);
            index2 = index(1)+1;
        else
            index2 = index(1);
            index1 = index2 - 1;
        end
        k = (t-final_path(index1,1))/(final_path(index2,1)-final_path(index1,1));
        sk = k * (final_path(index2,2)-final_path(index1,2)) + final_path(index1,2);
        path_s = [path_s;sk];
    end
end
inter_path = [path_t, path_s];
%{
figure;
hold on;
plot(path_t, path_s);
plot(final_path(:,1), final_path(:,2));
%}
smooth_s = smooth(path_s);
plot(path_t, smooth_s);

path_v = [];
smooth_v = [];
for i = 2:1:47
    path_v = [path_v; (path_s(i)-path_s(i-1))/0.1];
    smooth_v = [smooth_v; (smooth_s(i) - smooth_s(i-1))/0.1];
end

figure;
hold on;
plot(path_v);
plot(smooth_v);


