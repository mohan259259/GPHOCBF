function R = VecsToRot(p1, p2)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes two 3-vectors (the initial position p1 and the final position p2).
% Returns R (rotation matrix).
% 给定p1、p2，R的解不唯一，本函数选取旋转轴同时垂直于p1、p2的R。
% Example Input:
% 
% clear; clc;
% p1 = [1/sqrt(3); -1/sqrt(6); 1/sqrt(2)];
% p2 = [-0.5526; 0.4571; -0.6969];
% R = VecsToRot(p1, p2)
% 
% Output:
% R =
%   -0.0327   -0.3279   -0.9441
%   -0.2513   -0.9116    0.3253
%   -0.9674    0.2479   -0.0526

p1hat = Normalize(p1);
p2hat = Normalize(p2);
if NearZero(norm(p2hat-p1hat))
    R = eye(3);
elseif NearZero(norm(p2hat+p1hat))
    omg_0 = [p1(2); -p1(1); 0];
    if NearZero(norm(omg_0))
        omghat = [1; 0; 0];
    else
        omghat = Normalize(omg_0);
    end
    theta = pi;
    omgmat = VecToso3(omghat);
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
else
    omg = VecToso3(p1) * p2;
    omghat = Normalize(omg);
    theta = acos(transpose(p1hat) * p2hat);
    omgmat = VecToso3(omghat);
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
end
end