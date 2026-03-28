function phi = rosenbrock(u, p)
nu = length(u);
assert(length(p)==2);
a = p(1);
b = p(2);
phi = 0;
for i=1:nu-1
    phi = phi + b*(u(i+1)-u(i)^2)^2 + (a - u(i))^2;
end