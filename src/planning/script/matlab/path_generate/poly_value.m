function y = poly_value( coef, x , order)
%POLY_VALUE 此处显示有关此函数的摘要
%   此处显示详细说明

o_coef = coef;
size = length(o_coef);

for i = 1:1:order
    for j = 0:1:size-2
        o_coef(size-j) = o_coef(size-j-1)*(j+1); 
    end
    o_coef(i) = 0;
end
y = 0;
for i = 1:1:length(o_coef)
    y = y * x + o_coef(i);
end

end

