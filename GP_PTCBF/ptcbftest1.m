function psi_val = psi(t, x, p0, r, alpha0, T_s, t0_val, c_gain)
    % 从状态 x 中提取 q 和 qdot
    % 假设 x = [q(1); q(2); qdot(1); qdot(2); ...] 以 2D 为例
    q    = x(1:2);
    qdot = x(3:4);
    
    % 计算 h(q) = r^2 - ||q - p0||^2
    h_q = r^2 - norm(q - p0)^2;
    
    % 计算 dh(q)/dq = -2 * (q - p0)
    dh_dq = -2 * (q - p0);
    
    % 计算 z0(t) = ( T_pre^2 + c_gain * ( ((t - t0_val)^2) - (t - t0_val)*T_pre )^2 )
    %             / ( (T_pre + t0_val - t)^2 )
    z0_t = ( T_s^2 + c_gain * ( ((t - t0_val)^2) - (t - t0_val)*T_s )^2 ) ...
           / ( (T_s + t0_val - t)^2 );
    z1_t = ( T_psi^2 + c_gain * ( ((t - t0_val)^2) - (t - t0_val)*T_psi )^2 ) ...
           / ( (T_psi + t0_val - t)^2 );

    T_psi = (T_theta + T_s) / 2;


    % 组合得到 psi(t,x) = dh(q)/dq * qdot + z0(t)*alpha0*h(q)
    psi(t, x) = dh_dq' * qdot + z0_t * alpha0 * h_q;
end


dot_psi(t, x) = a_h(t, x) + b_h(x)' * (tau + d(x));

a_h(t, x) = d2h_dq2(q) * (qdot.^2) ...
    - dh_dq(q) * M_inv(q) * (C(x)*qdot + g(q)) ...
    + z0(t)*alpha0 * (dh_dq(q) * qdot) ...
    + dz0_dt(t)*alpha0 * h(q);

b_h(x) = M(q)'\ (dh_dq(q))';%这里的M还没定义，别忘了

if a_mu(t, x) + b_h(x)' * tau_n >= 0
    tau = tau_n;%名义控制器
else
    tau = tau_s;%不安全时用CBF进行安全控制
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
tau_s = (eye(n) - (b_h(x)*b_h(x)')/(b_h(x)'*b_h(x))) * tau_n - ((a_mu*b_h(x))/(b_h(x)'*b_h(x))* b_h(x));


a_mu = @(t,x) a_h(t,x) ...
           + b_h(x)' * mu_fn(x,theta_hat) ...
           - norm(b_h(x)) * (beta * sigma(x,theta_hat) + gamma) ...
           + z1(t) * alpha1 * psi(t, x);

gamma = (L_mu_theta + beta * L_sigma_theta) * 1 / Q_min_eig ) * (eps_bar_theta);

theta_hat = theta_n + delta_hat;

z1_t = ( T_psi^2 + c_gain * ( ((t - t0_val)^2) - (t - t0_val)*T_psi )^2 ) ...
           / ( (T_psi + t0_val - t)^2 );

    T_psi = (T_theta + T_s) / 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

tau_n



%%%%%%%%%%%%%%%%估计 L_{mu,theta} 的示例代码 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% 初始化历史记录变量
L_mu_theta_history = [];  % 用于记录每个时刻局部计算的 L_mu_theta

% 假设仿真循环开始
for i = 1:length(tspan)
    t = tspan(i);
    
    % 计算当前状态 current_x, GP预测 mu 等
    % ...（你的已有代码，这里省略）
    
    % 如果 GP 有数据，则计算有限差分得到 P_mu
    if gp.DataQuantity > 0
        delta_theta = 1e-6;  % 扰动步长
        
        % 对 SigmaL 的偏导数
        gp_l_plus = gp; 
        gp_l_plus.SigmaL = gp.SigmaL + delta_theta; 
        [mu_l_plus, ~, ~, ~, ~, ~, ~] = gp_l_plus.predict(current_x);
        dmu_dSigmaL = (mu_l_plus - mu) / delta_theta; 
        
        % 对 SigmaF 的偏导数
        gp_sigma_f_plus = gp;
        gp_sigma_f_plus.SigmaF = gp.SigmaF + delta_theta;
        [mu_sigma_f_plus, ~, ~, ~, ~, ~, ~] = gp_sigma_f_plus.predict(current_x);
        dmu_dSigmaF = (mu_sigma_f_plus - mu) / delta_theta;
        
        % 构造局部梯度向量
        P_mu = [dmu_dSigmaF, dmu_dSigmaL];
        
        % 计算当前局部的 Lipschitz 常数（2-范数）
        L_mu_theta_local = norm(P_mu, 2);
    else
        L_mu_theta_local = 0;
    end
    
    % 将当前时刻的局部值记录到历史数组中
    L_mu_theta_history = [L_mu_theta_history, L_mu_theta_local];
    
    % 其他仿真计算和状态更新
    % ...
end

%%仿真后，计算全局 Lipschitz 常数
L_mu_theta_global = max(L_mu_theta_history);
fprintf('Global L_{mu,theta} = %f\n', L_mu_theta_global);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%