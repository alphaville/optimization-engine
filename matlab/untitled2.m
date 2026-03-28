delta = 0.65;
g = 9.81;
phi = deg2rad(41);
c = 6.811 * 0.001;
k = 1885;
m = 0.462;
x1 = 0.5;
d = 0.42;

a = 5*g*sin(phi) / 7;
b = 5 * c / (7 * m);
gam = 5 * k / (7 * m);


sqrt(((delta - x1)^2 * (gam * (x1 - d) - a)) / b)