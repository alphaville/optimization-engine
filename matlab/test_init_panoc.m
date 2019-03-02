format long
u0 = [0.75; -1.4];
L = estimate_lip(u0);
gamma = 0.95/L
sigma = 0.49 * gamma * (1 - gamma * L)
[ell, grad] = costgrad(u0)

grad_step = u0 - gamma * grad;
half_step = 0.2*grad_step/norm(grad_step)

fpr = (u0-half_step)/gamma
norm(fpr)

%% BFGS (for testing)

rng(1);
n = 2; mem = 10; % dimensions

X = floor(100*randn(n, mem))/100; % X = [ x(k), x(k-1), ... ]
G = floor(100*randn(n, mem))/100; % G = [ g(k), g(k-1), ... ]

% Sk = [s(k-1), ..., s(0)], where s(k-1) = x(k) - x(k-1)
S = diff(X')'; 
% Yk = [y(k-1), ..., y(0)], where y(k-1) = g(k) - g(k-1)
Y = diff(G')';


v = [1;1];

% We may compute H(k) using Sk and Yk as follows
k = 1; Hk =  bfgs(H0, k, Y, S);
d_bfgs = Hk * v

d = lbfgs(H0, v, Y(:,1:1), S(:,1:1))


%% Test LBFGS

% --- First 
S = [0.1, 0.2, -0.3]';
Y = [-0.5, 0.6, -1.2]';

va = [-3.1, 1.5, 2.1]'; 
gamma = (Y(:,1)'*S(:,1))/(Y(:,1)'*Y(:,1));
dir_a_correct = [-1.100601247872944, -0.086568349404424, 0.948633011911515]';
[d,a,rho,beta,q] = lbfgs(gamma*eye(3), va, Y, S);
assert ( norm(dir_a_correct - d) < 1e-10 )

% --- Second
S = [[0.09, -0.01, -0.14]', S];
Y = [[-0.25, 0.30, -0.70]', Y];

vb = [-3.1, 1.5, 2.1]';
gamma = (Y(:,1)'*S(:,1))/(Y(:,1)'*Y(:,1));
dir_b_correct = [-1.814749861477524, 0.895232314736337, 1.871795942557546]';
d = lbfgs(gamma*eye(3), va, Y, S);
assert ( norm(dir_b_correct - d) < 1e-10 )


% --- Third
S = [[0.2,  0.2, -0.4]', S];
Y = [[-1.5, 2.6, -1.2]', Y];

vc = [1.1, 0.2, -0.3]';
gamma = (Y(:,1)'*S(:,1))/(Y(:,1)'*Y(:,1));
dir_c_correct = [1.025214973501680, 0.070767249318312, -1.444856343354091]';
d = lbfgs(gamma*eye(3), vc, Y, S);
assert ( norm(dir_c_correct - d) < 1e-10 )


% --- Fourth 
S = [[0.1, 0.2, -0.4]', S]; S(:, end) = [];
Y = [[-1.5, 2.8, -1.2]', Y]; Y(:, end) = [];

vd = [-2.0, 0.2, -0.3]';
gamma = (Y(:,1)'*S(:,1))/(Y(:,1)'*Y(:,1));
dir_d_correct = [-0.933604237447365, -0.078865807539102, 1.016318412551302]';
[dd,ad,rhod,betad,qd] = lbfgs(gamma*eye(3), vd, Y, S);
assert ( norm(dir_d_correct - dd) < 1e-10 )
