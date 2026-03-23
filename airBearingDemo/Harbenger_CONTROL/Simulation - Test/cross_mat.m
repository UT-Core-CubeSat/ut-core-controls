function [x_cross] = cross_mat(x)
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    x_cross = [0 -x3 x2;
               x3 0 -x1;
               -x2 x1 0];
end