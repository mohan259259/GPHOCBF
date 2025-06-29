function [q, s, h] = AxisToScrew(S)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes the corresponding normalized screw axis.
% Returns q: a point lying on the screw axis,
%         s: a unit vector in the direction of the screw axis, 
%         h: the pitch of the screw axis.
% Example Input:
% 
% clear; clc;
% S = [0; 0; 1; 0; -3; 2];
% [q, s, h] = AxisToScrew(S)
% 
% Output:
% q =
% 
%      3
%      0
%      0
% 
% 
% s =
% 
%      0
%      0
%      1
% 
% 
% h =
% 
%      2

s = S(1: 3);
h = transpose(s) * S(4: 6);
X = S(4: 6) - h * s;
q1 = -X(2) / S(3);
q2 = X(1) / S(3);
q = [q1; q2; 0];
end