function test_prescribed_time_CBF_record
    % 清除环境
    clc; clear; close all;
    
    % --- 声明全局变量用于记录控制律中 nominal 力矩和判别式 ---
    global tau_nom_history lhs_check_history control_time_history control_counter;
    tau_nom_history = [];       % 用于记录 nominal 力矩 tau_n（3×N_entries）
    lhs_check_history = [];     % 用于记录判别式值 (1×N_entries)
    control_time_history = [];  % 用于记录对应时间 (1×N_entries)
    control_counter = 0;        % 控制律调用次数计数器

    % =========== 用户/问题设置 ===========
    % 1) 安全球: 半径 r, 球心 p0 (3×1)
    r    = 1.0;
    p0   = [0; 0; 0];

    % 2) 预设时间参数
    T_s     = 2;          % safety “deadline”
    T_theta = 1;
    T_psi   = (T_s + T_theta) / 2;  
    c_gain  = 1.0;
    t0_val  = 0.0;  

    % 3) CBF控制参数
    alpha0  = 1.0;
    alpha1  = 1.0;
    beta    = 1.0;
    gamma   = 1.0;

    % 4) 名义PD增益 (3×3)
    Kp      = 10.0 * eye(3);
    Kd      = 5.0 * eye(3);

    % 5) “GP” mean & std. dev. (mocked)
    mu_fn   = @(x,theta_hat) ones(3,1); % 输出3×1向量
    sigma_fn= @(x,theta_hat) 0.5;       % 标量

    % 6) PD 期望位置 (q_des in 3D)
    q_des   = [3; 3; 3];

    % 7) 初始状态: q(0) = [3;3;3], dq(0) = [0;0;0]
    q0      = [3; 3; 3];
    dq0     = [0; 0; 0];
    x0      = [q0; dq0];  % 状态为6×1

    % 8) 积分器设置
    opts    = odeset('RelTol', 1e-7, 'AbsTol', 1e-8);

    % ========== 分段积分：避开 t = 1.5 和 t = 2 ==========
    % 第一段：0 → 1.49
    tspan1 = [0, 1.49];
    [t1, x1] = ode45(@robotDynamics, tspan1, x0, opts);
    
    % 第二段：1.51 → 1.99
    x_mid = x1(end, :);
    tspan2 = [1.51, 1.99];
    [t2, x2] = ode45(@robotDynamics, tspan2, x_mid, opts);
    
    % 第三段：2.01 → 10
    x_mid2 = x2(end, :);
    tspan3 = [2.01, 10];
    [t3, x3] = ode45(@robotDynamics, tspan3, x_mid2, opts);

    % ========== 合并两段结果 ==========
    tSol = [t1; t2; t3];
    xSol = [x1; x2; x3];

    % 绘图
    figure('Position', [100, 100, 1200, 600]);
    hold on; grid on;
    % 计算 h(q(t))
    hVals = arrayfun(@(i) hFun(xSol(i, 1:3).'), 1:length(tSol));
    plot(tSol, hVals, 'LineWidth', 2, 'DisplayName', 'h(q(t))');
    plot([T_s T_s], ylim, 'r--', 'DisplayName', 't = T_s');
    xlabel('Time (s)'); ylabel('h(q(t))');
    title('Barrier Function vs Time (3D)');
    legend('Location', 'Best');

    % 打印最终结果
    fprintf('===========================\n');
    fprintf('Final time = %.2f,  h(q(tEnd)) = %.4f\n', tSol(end), hFun(xSol(end, 1:3).'));
    idxTs = find(tSol >= T_s, 1);
    fprintf('Check near T_s = %.2f => index where t >= T_s: %d\n', T_s, idxTs);
    fprintf('===========================\n');

    % 导出全局变量到 base workspace
    assignin('base', 'tau_nom_history', tau_nom_history);
    assignin('base', 'lhs_check_history', lhs_check_history);
    assignin('base', 'control_time_history', control_time_history);
    disp('记录的控制变量已保存到工作区：tau_nom_history, lhs_check_history, control_time_history');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% 以下为嵌套函数: 机器人动力学, 控制律, 辅助函数 %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function dxdt = robotDynamics(t, x)
        q  = x(1:3);
        dq = x(4:6);
        tau = controlLaw(t, x);
        ddq = tau;  % 系统模型 M = I(3)
        dxdt = [dq; ddq];
    end

    function tau = controlLaw(t, x)
        q  = x(1:3);
        dq = x(4:6);

        % 计算 psi, a_h, b_h
        psi_val = psiFun(t, q, dq);
        ah_val  = a_hFun(t, q, dq);
        bh_val  = b_hFun(q);  % 3×1

        % 模拟 GP: 均值和标准差
        mu_val    = mu_fn(x, []);
        sigma_val = sigma_fn(x, []);

        % 计算安全调整项 amu_val
        amu_val = ah_val + bh_val' * mu_val ...
                  - norm(bh_val) * (beta * sigma_val + gamma) ...
                  + z1Fun(t) * alpha1 * psi_val;

        % 名义 PD 控制律: tau_n
        tau_n = -Kp * (q - q_des) - Kd * dq;  % 3×1

        % 判别式: amu_val + bh_val' * tau_n
        lhs_check = amu_val + bh_val' * tau_n;

        % 记录当前时刻的信息
        control_counter = control_counter + 1; %#ok<NASGU>
        control_time_history(control_counter) = t;
        tau_nom_history(:, control_counter)    = tau_n;
        lhs_check_history(control_counter)     = lhs_check;

        fprintf('Time: %.4f, lhs_check: %.4f\n', t, lhs_check);

        if lhs_check >= 0
            tau = tau_n;
        else
            % 安全控制: QP 近似解
            denom = bh_val' * bh_val;
            I3 = eye(3);
            proj = I3 - (bh_val * bh_val') / denom;
            tau_s = proj * tau_n - (amu_val / denom) * bh_val;
            tau = tau_s;
        end
    end

    % Barrier 函数: h(q) = r^2 - ||q - p0||^2
    function val = hFun(q_3)
        val = r^2 - norm(q_3 - p0)^2;
    end

    % psiFun(t,q,dq) = dh/dq' * dq + alpha0 * z0Fun(t) * h(q)
    function val = psiFun(t, q, dq)
        val = dhdqFun(q)' * dq + alpha0 * z0Fun(t) * hFun(q);
    end

    % a_hFun(t,q,dq) = ... 详见主文
    function val = a_hFun(t, q, dq)
        H_ = d2hdq2Fun(q);        
        term1 = dq.' * H_ * dq;
        term2 = alpha0 * z0Fun(t) * (dhdqFun(q)' * dq);
        term3 = alpha0 * dz0Fun(t) * hFun(q);
        val = term1 + term2 + term3;
    end

    % b_hFun(q) = dh/dq
    function bh = b_hFun(q)
        bh = dhdqFun(q);
    end

    function grad_h = dhdqFun(q_3)
        grad_h = -2 * (q_3 - p0);
    end

    function H3 = d2hdq2Fun(~)
        H3 = -2 * eye(3);
    end

    function val = z0Fun(t)
        numerator   = T_s^2 + c_gain * (((t - t0_val)^2 - (t - t0_val) * T_s)^2);
        denominator = (T_s - t)^2;
        val = numerator / denominator;
    end

    function val = dz0Fun(t)
        A = (t - t0_val);
        B = (T_s - t);
        part1 = c_gain * 2 * (A^2 - A * T_s) * (2 * A - T_s) * B^2;
        part2 = (T_s^2 + c_gain * ((A^2 - A * T_s)^2)) * (-2 * B);
        val = (part1 - part2) / (B^4);
    end

    function val = z1Fun(t)
        numerator   = T_psi^2 + c_gain * (((t - t0_val)^2 - (t - t0_val) * T_psi)^2);
        denominator = (T_psi - t)^2;
        val = numerator / denominator;
    end

end
