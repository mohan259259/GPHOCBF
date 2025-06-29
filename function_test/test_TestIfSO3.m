clear; clc;
addpath(genpath('..\'));
% mat = [1,  0,  0;
%        0, -1,  0;
%        0,  0, -1];
mat = [1,  0,  0;
       0,  0,  1;
       0, -1,  0];

judge = TestIfSO3(mat)