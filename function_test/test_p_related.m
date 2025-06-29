clear;
clc;
addpath(genpath('..\'));

syms t l1 l2 l3 d1 d2 lf;

q = [1*sin(t); 2*sin(t); 3*cos(t); 4*sin(t); 5*cos(t); 6*sin(t); 7*sin(t)];
qdot = diff(q);

[p, pdot, Jaco, K] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lf);

check1 = simplify(pdot - diff(p))
check2 = simplify(pdot - Jaco * qdot)