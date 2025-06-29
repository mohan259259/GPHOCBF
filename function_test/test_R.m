clear; clc;
addpath(genpath('..\'));
omghat_x = [1; 0; 0];
omghat_y = [0; 1; 0];
omghat_z = [0; 0; 1];

theta = [0.44; 0.16; -90.11] * pi / 180;

p = 0 * pi / 180;

R = MatrixExp3(VecToso3(omghat_x * theta(1))) * MatrixExp3(VecToso3(omghat_y * theta(2))) ...
    * MatrixExp3(VecToso3(omghat_z * theta(3)));

R = R * MatrixExp3(VecToso3(omghat_z * p))