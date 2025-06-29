function [h_val, grad_h, Lf_h, LgLf_h, xi_val, dot_xi_val, bar_alpha_val] = computeLieDerivatives(...
    qdot, p0, r_s, t, T_d, omega, alpha, p, J, M)
    % qdot: 7x1 关节速度
    % p0: 3x1 安全区球心
    % r_s: 安全区半径
    % t: 当前时间
    % T_d: 规定安全时间
    % omega: 时间函数参数
    % alpha: 安全修正增益
    % p 机器人末端位置
    % J Jacobian
    % M 机器人质量矩阵

    % 2. 计算安全函数值 h(q)
    h_val = r_s^2 - norm(p - p0)^2;
    
    % 3. 计算梯度 grad_h = ∇h(q) = -2*(p - p0)'*J  (1x7 向量)
    grad_h = -2 * (p - p0)' * J;
    
    % 4. 一阶 Lie 导数 L_f h = grad_h * qdot
    Lf_h = grad_h * qdot;
    
    % 5. 计算机器人质量矩阵 M(q)（假设你有函数 Panda_MassMatrix）
    % M = Panda_MassMatrix(q);  % 7x7矩阵
    M_inv = inv(M);
    
    % 6. 控制输入相关的 Lie 导数 L_g L_f h = grad_h * M_inv
    LgLf_h = grad_h * M_inv;  % 1x7向量
    
    % 7. 时间修正函数 xi(t) 和其导数 dot_xi(t)
    % 定义 xi(t) = exp(omega*(T_d - t)) - 1, t in [0, T_d)
    if t < T_d
        xi_val = exp(omega*(T_d - t)) - 1;
        dot_xi_val = -omega * exp(omega*(T_d - t));
    else
        xi_val = exp(omega*(t - T_d));
        dot_xi_val = omega * exp(omega*(t - T_d));
    end
    
    % 8. 根据 h(q) 的正负计算修正项 bar_alpha
    if h_val < 0
        bar_alpha_val = abs(dot_xi_val)/xi_val * h_val + alpha * h_val;
    else
        bar_alpha_val = alpha * h_val;
    end
end
