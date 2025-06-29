clear; clc;
addpath(genpath('..\'));

l1 = 0.333;
l2 = 0.316;
l3 = 0.384;
d1 = 0.0825;
d2 = 0.088;
lf = 0.107 + 0.1034;

% q = [0; 0; 0; -90; 0; 90; 0] * pi / 180;
q = [11; 22; 33; -44; 55; 66; 77] * pi / 180;
% q = [0; 0; 0; -30; 0; 359.5; 0] * pi / 180;

Tsb = Panda_syms_FK(q, l1, l2, l3, d1, d2, lf);
q7 = q(7);
qa = q;

Q = Panda_syms_IK(Tsb, l1, l2, l3, d1, d2, lf, q7, qa);
Q * 180 / pi

% Tsb - Panda_syms_FK([Q(:, 4); q7], l1, l2, l3, d1, d2, lf)