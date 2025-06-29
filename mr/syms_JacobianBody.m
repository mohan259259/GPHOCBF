function Jb = syms_JacobianBody(Blist, thetalist)
% *** CHAPTER 5: VELOCITY KINEMATICS AND STATICS ***
% 对应JacobianBody函数，实现符号运算
% Jb = simplify(syms_JacobianBody(Blist, thetalist))

Jb = Blist;
T = eye(4);
for i = length(thetalist) - 1: -1: 1
    B = -Blist(:, i + 1);
    omg = B(1: 3);
    v = B(4: 6);
    theta = thetalist(i + 1);
    omgmat = VecToso3(omg);
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    p = (eye(3) * theta + (1 - cos(theta)) * omgmat ...
          + (theta - sin(theta)) * omgmat * omgmat) ...
            * v;
    X = [R, p; 0, 0, 0, 1];
    T = T * X;
    Jb(:, i) = Adjoint(T) * Blist(:, i);
end
end