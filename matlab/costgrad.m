function [cost, grad] = costgrad(u)
Q = [1 1;1 2];
q = [1;-1];
c = 3;
grad = Q*u + q;
cost = 0.5*u'*Q*u + q'*u + c;