function [r,a,rho,beta,q] = lbfgs(H0, v, Y, S)
%LBFGS performs a simple L-BFGS update
%
% Input arguments:
% - H0: Initial Hessian estimate (matrix); typical choice: H0 = (s'y)/(y'y)
% where (s,y) is the most recent pair of s and y
% - v: the vector on which the L-BFGS Hessian estimate should be applied;
% the function will return r = Hk*v, for given v
% - Y and S: buffers: Y = [y(k-1), ..., y(0)], where y(k-1) = g(k) - g(k-1)
% and Sk = [s(k-1), ..., s(0)], where s(k-1) = x(k) - x(k-1)
%
% Output arguments:
% - r = Hk*v
%
q = v;
sz = size(Y,2);

rho = zeros(sz, 1);
a = zeros(sz, 1);
for i=1:sz
    si = S(:, i);
    yi = Y(:, i);
    rho(i) = 1./(si'*yi);
    a(i) = rho(i) * si'*q;
    q = q - a(i)*yi;
end
r = H0*q;
for i=sz:-1:1
    yi = Y(:, i);
    si = S(:, i);
    beta = rho(i) * yi' * r;
    r = r + si*(a(i) - beta);
end
