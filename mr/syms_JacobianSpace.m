function Js = syms_JacobianSpace(Slist, thetalist)
% *** CHAPTER 5: VELOCITY KINEMATICS AND STATICS ***
% 对应JacobianSpace函数，实现符号运算
% Js = simplify(syms_JacobianSpace(Slist, thetalist))

Js = Slist;
T = eye(4);
for i = 2: length(thetalist)
    S = Slist(:, i - 1);
    omg = S(1: 3);
    v = S(4: 6);
    theta = thetalist(i - 1);
    omgmat = VecToso3(omg);
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    p = (eye(3) * theta + (1 - cos(theta)) * omgmat ...
          + (theta - sin(theta)) * omgmat * omgmat) ...
            * v;
    X = [R, p; 0, 0, 0, 1];
    T = T * X;
    Js(:, i) = Adjoint(T) * Slist(:, i);
end
end