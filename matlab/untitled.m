figure(1);
omega = 0.07;
h = 0.1;
ts = 0:h:100;
us = sin(omega * ts);
xs = cumsum(us) * h;
plot(xs)
hold on