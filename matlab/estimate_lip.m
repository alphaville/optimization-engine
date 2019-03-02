function L = estimate_lip(u)
delta = 1e-10;
epsilon = 1e-10;
h = max(delta, epsilon*u);
norm_h = norm(h);
[~, grad_u] = costgrad(u);
[~, grad_u_h] = costgrad(u+h);
L = norm(grad_u_h - grad_u) / norm_h;