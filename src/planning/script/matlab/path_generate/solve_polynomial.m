function coefficient = solve_polynomial(t0, t1, x0, x1, dx0, dx1, ddx0, ddx1)
%SOLVE_POLYNOMIAL 此处显示有关此函数的摘要
%   此处显示详细说明

A = [t0^5, t0^4, t0^3, t0^2, t0, 1; ...
     t1^5, t1^4, t1^3, t1^2, t1, 1; ...
     5*t0^4, 4*t0^3, 3*t0^2, 2*t0, 1, 0; ...
     5*t1^4, 4*t1^3, 3*t1^2, 2*t1, 1, 0; ...
     20*t0^3, 12*t0^2, 6*t0, 2, 0, 0; ...
     20*t1^3, 12*t1^2, 6*t1, 2, 0, 0];

B = [x0; x1; dx0; dx1; ddx0; ddx1];

coefficient = inv(A)*B;

end

