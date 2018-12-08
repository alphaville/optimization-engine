function H = bfgs(H0, k, Y, S)
n = size(H0, 1);
H = H0;

for i = 1:k
    s = S(:, i);
    y = Y(:, i);
    rho = 1/(s' * y);
    V = eye(n) - rho*y*s';
    H = V' * H * V + rho * s * s';
end
