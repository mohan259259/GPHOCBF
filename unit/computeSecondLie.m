function Lf2_h = computeSecondLie(qdot, qddot_nom, p0, r_s, p, J, M)
    % 计算二阶Lie导数 L_f^2 h(q)
    % 对于 h(q) = r_s^2 - ||p(q) - p0||^2，
    % L_f h = grad_h * qdot，
    % L_f^2 h = d/dt(L_f h) = (dgrad_h/dq * qdot)*qdot + grad_h * qddot_nom.
    % 此处为了示例，假设 (dgrad_h/dq * qdot) 的影响较小，简化为:
    [~, grad_h, ~, ~, ~, ~, ~] = computeLieDerivatives(qdot, p0, r_s, 0, 0, 1, 5, p, J, M);
    Lf2_h = grad_h * qddot_nom;  % 简单近似
end

