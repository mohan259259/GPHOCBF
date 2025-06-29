function theta = SolveThetaProblem(v1, v2, omghat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% ����v1��v2������ģ����ͬ���͵�λת�ᣨ���������������ת�ǡ�
% Example Input:
% 
% clear; clc;
% v1 = [1; 0; 1];
% v2 = [0; 1; 1];
% omghat = [2/3; 2/3; 1/3];
% theta = SolveThetaProblem(v1, v2, omghat)
% 
% Output:
% theta =
%
%    -1.5708

v1hat = Normalize(v1 - transpose(v1) * omghat * omghat);
v2hat = Normalize(v2 - transpose(v2) * omghat * omghat);
if transpose(VecToso3(v1) * v2) * omghat >= 0
    theta = acos(transpose(v1hat) * v2hat);
else
    theta = -acos(transpose(v1hat) * v2hat);
end
end