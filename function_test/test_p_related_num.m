clear;
clc;
addpath(genpath('..\'));

l1 = 0.333;
l2 = 0.316;
l3 = 0.384;
d1 = 0.0825;
d2 = 0.088;
lf = 0.107;

q = [1; 2; 3; -4; 5; 6; 7];
qdot = [1; 2; 3; -4; 5; 6; 7];
qddot = [1; 2; 3; -4; 5; 6; 7];

% q = q';
% qdot = qdot';

[p, pdot, Jaco, K] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lf)
pddot = Panda_pddot(q, qdot, qddot, l1, l2, l3, d1, d2, lf)