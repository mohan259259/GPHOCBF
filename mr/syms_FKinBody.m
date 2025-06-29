function T = syms_FKinBody(M, Blist, thetalist)
% *** CHAPTER 4: FORWARD KINEMATICS ***
% 对应FKinBody函数，实现符号运算
% T = simplify(syms_FKinBody(M, Blist, thetalist))

T = M;
for i = 1: size(thetalist)
    B = Blist(:, i);
    omg = B(1: 3);
    v = B(4: 6);
    theta = thetalist(i);
    omgmat = VecToso3(omg);
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    p = (eye(3) * theta + (1 - cos(theta)) * omgmat ...
          + (theta - sin(theta)) * omgmat * omgmat) ...
            * v;
    X = [R, p; 0, 0, 0, 1];
    T = T * X;
end
end