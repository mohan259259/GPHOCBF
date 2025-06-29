function T = syms_FKinSpace(M, Slist, thetalist)
% *** CHAPTER 4: FORWARD KINEMATICS ***
% 对应FKinSpace函数，实现符号运算
% T = simplify(syms_FKinSpace(M, Slist, thetalist))

T = M;
for i = size(thetalist): -1: 1
    S = Slist(:, i);
    omg = S(1: 3);
    v = S(4: 6);
    theta = thetalist(i);
    omgmat = VecToso3(omg);
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    p = (eye(3) * theta + (1 - cos(theta)) * omgmat ...
          + (theta - sin(theta)) * omgmat * omgmat) ...
            * v;
    X = [R, p; 0, 0, 0, 1];
    T = X * T;
end
end