function kill_parametric_optimizer(ip_address, port)
%echoudp('on', port)
u = udp(ip_address, port);
fopen(u);
fwrite(u, 'x');
X = fread(u, 128);
disp(char(X'));
fclose(u);
