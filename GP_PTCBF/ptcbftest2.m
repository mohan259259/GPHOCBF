function main_simulation_3d_switch()
    %% 仿真参数设置
    T_end   = 50;         % 仿真总时长（秒）
    dt      = 0.01;       % 时间步长
    tspan   = 0:dt:T_end; % 时间序列
    
    % 状态初始条件：x = [q; qdot]，这里 q, qdot 均为 3 维向量
    x0 = [0; 0; 0; 0; 0; 0];  
    
    % 目标与安全区：安全区为以 p0 为中心，半径 r 的球
    p0 = [1; 1; 1];   % 球心，也是目标点
    r  = 0.5;         % 半径：当 h(q)=r^2 - ||q-p0||^2>0 时，q 处于安全区内
    
    % CBF 相关参数
    alpha0  = 1;   
    alpha1  = 1;
    T_s     = 2;
    T_theta = 1;
    t0_val  = 0;
    c_gain  = 1;
    
    % PD 控制器参数（名义控制器用），3×3 矩阵
    Kp = eye(3)*5;
    Kd = eye(3)*2;
    
    % 其它占位参数（如 GP 相关，这里用占位值）
    beta   = 1;
    gamma  = 1;
    
    % 初始 GP 参数（占位）
    theta_n  = eye(3,1);
    delta_hat = eye(3,1);
    theta_hat = theta_n + delta_hat;
    
    % 历史数据初始化：状态、控制输入、 h(q) 记录
    x_history    = zeros(6, length(tspan));
    tau_history  = zeros(3, length(tspan));
    h_history    = zeros(1, length(tspan));
    switch_vals  = zeros(1, length(tspan)); % 记录每个时刻 S(t) = a_mu + b_h'*tau_n
    tau_choice   = zeros(1, length(tspan)); % 记录采用的控制器类型：1--nominal; 0--安全
    
    x_history(:,1) = x0;
    x_current = x0;
    
    %% 主循环：计算控制、状态更新，并记录判断过程
    for i = 1:length(tspan)
        t = tspan(i);
        
        % 计算当前 q 和 h(q)
        q_current = x_current(1:3);
        h_current = r^2 - norm(q_current - p0)^2;
        h_history(i) = h_current;
        
        % --- 1. 计算 ψ(t,x) ---
        psi_val = psi(t, x_current, p0, r, alpha0, T_s, T_theta, t0_val, c_gain);
        
        % --- 2. 计算 a_h(t,x) ---
        a_h_val = a_h(t, x_current, p0, r, alpha0, T_s, T_theta, t0_val, c_gain);
        
        % --- 3. 计算 b_h(x) ---
        bh = b_h(x_current, p0);  % bh 为 3×1 列向量
        
        % --- 4. 计算 z₁(t)  ---
        T_psi = (T_theta + T_s) / 2;
        z1_t = ( T_psi^2 + c_gain * (((t-t0_val)^2 - (t-t0_val)*T_psi)^2) ) / ((T_psi + t0_val - t)^2);
        
        % --- 5. 计算 a_mu(t,x) ---
        % a_mu = a_h + b_h' * mu_fn(x,theta_hat) - norm(b_h)*(β*σ(x,theta_hat) + γ) + z₁(t)*α₁*ψ(t,x)
        % 这里 mu_fn 返回 3×1 零向量，sigma 返回 0
        a_mu_val = a_h_val + (bh' * mu_fn(x_current, theta_hat)) ...
                   - norm(bh)*(beta * sigma(x_current, theta_hat) + gamma) ...
                   + z1_t * alpha1 * psi_val;
               
        % --- 6. 计算名义控制器 τ_n ---
        tau_n = nominal_controller_PD(x_current, p0, Kp, Kd);
        
        % --- 7. 判断安全性条件，并记录判断值 S(t) ---
        % S(t) = a_mu + b_h' * τ_n
        S_val = a_mu_val + bh' * tau_n;
        switch_vals(i) = S_val;
        
        if (S_val >= 0)
            tau = tau_n;
            tau_choice(i) = 1;  % 表示此时用的是 τ_n
        else
            tau = safe_controller(tau_n, a_mu_val, bh);
            tau_choice(i) = 0;  % 表示此时用的是安全控制器 τ_s
        end
        
        tau_history(:,i) = tau;
        
        % --- 8. 状态更新 ---
        q     = x_current(1:3);
        qdot  = x_current(4:6);
        M_val = M(q);
        C_val = C(x_current);
        g_val = g(q);
        qddot = inv(M_val) * (tau - C_val*qdot - g_val);
        x_dot = [qdot; qddot];
        x_next = x_current + dt * x_dot;
        if i < length(tspan)
            x_history(:,i+1) = x_next;
        end
        x_current = x_next;
    end
    
    %% 绘图
    % 1. 绘制 3D 轨迹及安全球
    figure;
    plot3(x_history(1,:), x_history(2,:), x_history(3,:), 'LineWidth', 2);
    hold on;
    plot3(p0(1), p0(2), p0(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    [Xs, Ys, Zs] = sphere(20);
    surf(r*Xs + p0(1), r*Ys + p0(2), r*Zs + p0(3), 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    xlabel('q_1'); ylabel('q_2'); zlabel('q_3');
    title('3D 系统轨迹及安全球');
    grid on;
    axis equal;
    

    
    % 3. 绘制 h(q) 随时间变化（观察安全区域）
    figure;
    plot(tspan, h_history, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('h(q)');
    title('h(q) 随时间变化 (应保持 h(q)>0)');
    grid on;
    
    % 4. 绘制判断过程 S(t) = a_mu + b_h''*tau_n 随时间变化，并标出切换点
    figure;
    plot(tspan, switch_vals, 'LineWidth', 2);
    hold on;
    yline(0, '--r','Threshold 0');
    xlabel('Time (s)');
    ylabel('S(t) = a_{\mu} + b_h^T τ_n');
    title('判断条件 S(t) 随时间变化');
    grid on;
 
    
    fprintf('仿真结束。\n');
end

%% ------- 子函数定义 --------

function psi_val = psi(t, x, p0, r, alpha0, T_s, T_theta, t0_val, c_gain)
    % psi: 计算 ψ(t,x) = (dh/dq)*qdot + z₀(t)*α₀*h(q)
    q    = x(1:3);
    qdot = x(4:6);
    h_q  = r^2 - norm(q - p0)^2;
    dh_dq = -2*(q - p0)';  % 1×3 行向量
    z0_t = ( T_s^2 + c_gain * (((t - t0_val)^2 - (t - t0_val)*T_s)^2) ) / ((T_s + t0_val - t)^2);
    psi_val = dh_dq * qdot + z0_t * alpha0 * h_q;
end

function ah = a_h(t, x, p0, r, alpha0, T_s, T_theta, t0_val, c_gain)
    % a_h: 计算 aₕ(t,x)
    q    = x(1:3);
    qdot = x(4:6);
    h_q  = r^2 - norm(q - p0)^2;
    dh_dq = -2*(q - p0)';   % 1×3
    term1 = -2*(qdot' * qdot);
    
    M_val = M(q);
    C_val = C(x);
    g_val = g(q);
    term2 = - dh_dq * inv(M_val) * (C_val*qdot + g_val);
    



    %denom = (T_s + t0_val - t)^2;
    %if denom < 1e-6
    %   denom = 1e-6;
    %end
    %z0_t = (T_s^2 + c_gain * (((t - t0_val)^2 - (t - t0_val)*T_s)^2)) / denom;

    z0_t = ( T_s^2 + c_gain * (((t - t0_val)^2 - (t - t0_val)*T_s)^2) ) / ((T_s + t0_val - t)^2);
    dt_small = 1e-6;
    z0_t_next = ( T_s^2 + c_gain * ((((t+dt_small) - t0_val)^2 - ((t+dt_small) - t0_val)*T_s)^2) ) / ((T_s + t0_val - (t+dt_small))^2);
    dz0 = (z0_t_next - z0_t) / dt_small;
    
    term3 = z0_t * alpha0 * (dh_dq * qdot);
    term4 = dz0 * alpha0 * h_q;
    ah = term1 + term2 + term3 + term4;
end

function bh = b_h(x, p0)
    % b_h: 计算 bₕ(x) = inv(M(q)')*(dh/dq)'
    q = x(1:3);
    dh_dq = -2*(q - p0)';  % 1×3
    M_val = M(q);
    bh = (M_val') \ (dh_dq');  % 3×1 列向量
end

function tau_n = nominal_controller_PD(x, p0, Kp, Kd)
    % nominal_controller_PD: PD + 重力补偿名义控制器
    q    = x(1:3);
    qdot = x(4:6);
    e    = q - p0;
    M_val = M(q);
    C_val = C(x);
    g_val = g(q);
    u_PD  = - Kp * e - Kd * qdot;
    tau_n = M_val * u_PD + C_val * qdot + g_val;
end

function tau_s = safe_controller(tau_n, a_mu_val, bh)
    % safe_controller: 基于 CBF 调整，保证安全
    n = length(tau_n);
    bh_norm_sq = bh' * bh;
    tau_s = (eye(n) - (bh*bh')/bh_norm_sq)*tau_n - (a_mu_val/bh_norm_sq)*bh;
end

%% 系统动力学及占位模型函数

function M_val = M(q)
    % 简单的 3×3 惯性矩阵
    M_val = eye(3);
end

function C_val = C(x)
    % 简化为零矩阵
    C_val = zeros(3);
end

function g_val = g(q)
    % 重力向量（此处设为零，便于观察控制效果）
    g_val = zeros(3,1);
end

function mu_val = mu_fn(x, theta_hat)
    % 占位函数：返回 3×1 零向量
    mu_val = eye(3,1);
end

function sigma_val = sigma(x, theta_hat)
    % 占位函数
    sigma_val = 1;
end

%% 启动仿真
main_simulation_3d_switch();