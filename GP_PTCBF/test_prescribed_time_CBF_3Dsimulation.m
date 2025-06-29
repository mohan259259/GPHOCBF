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
    r    = 1.0;  % 半径
    p0   = [0; 0; 0];  % 球心位置

    % 预设时间参数
    T_s     = 2;          % safety “deadline”
    T_theta = 1;
    T_psi   = (T_s + T_theta) / 2;  
    c_gain  = 1.0;
    t0_val  = 0.0;  

    % CBF控制参数
    alpha0  = 1.0;
    alpha1  = 1.0;
    beta    = 1.0;
    gamma   = 1.0;

    % 名义PD增益 (3×3)
    Kp      = 10.0 * eye(3);
    Kd      = 5.0 * eye(3);

    % “GP” mean & std. dev. (mocked)
    mu_fn   = @(x,theta_hat) ones(3,1);  % 输出3×1向量
    sigma_fn= @(x,theta_hat) 0.5;        % 标量

    % PD 期望位置 (q_des in 3D)
    q_des   = [3; 3; 3];

    % 初始状态: q(0) = [3;3;3], dq(0) = [0;0;0]
    q0      = [3; 3; 3];  % 末端执行器位置
    dq0     = [0; 0; 0];  % 末端执行器速度
    x0      = [q0; dq0];  % 状态为6×1

    % 积分器设置
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

    % 合并结果
    tSol = [t1; t2; t3];
    xSol = [x1; x2; x3];

    % --- 打印并绘制 q(t) 在 10 秒内的位置轨迹 ---
    q0_trajectory_values = [];  % 用于存储每个时间步的 q(t)

    for i = 1:length(tSol)
        q_current = xSol(i, 1:3).';  % 提取当前时刻的 q (3×1)
        q0_trajectory_values = [q0_trajectory_values, q_current];  % 将位置添加到轨迹中
        fprintf('At time %.2f, q(t) = [%.4f, %.4f, %.4f]\n', tSol(i), q_current(1), q_current(2), q_current(3));
    end

    % --- 打印 q0 在 t=0 和 t=10s 时的具体位置坐标 ---
    initial_q0 = q0;  % t=0 时的 q0
    final_q0 = q0_trajectory_values(:, end);  % t=10s 时的 q0
    fprintf('At t = 0s, q0 = [%.4f, %.4f, %.4f]\n', initial_q0(1), initial_q0(2), initial_q0(3));
    fprintf('At t = 10s, q0 = [%.4f, %.4f, %.4f]\n', final_q0(1), final_q0(2), final_q0(3));

    % 绘制 q(t) 的轨迹
    figure('Position', [100, 100, 1200, 600]);
    plot3(q0_trajectory_values(1, :), q0_trajectory_values(2, :), q0_trajectory_values(3, :), 'LineWidth', 2);
    hold on;

    % 绘制半径1的渐变色透明球体（安全区）
    [X, Y, Z] = sphere(50);  % 生成球体数据
    % 使用渐变色绘制球体
    surf(X, Y, Z, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'interp');
    colormap jet;  % 设置渐变色
    caxis([0 1]);  % 设置颜色映射的范围

    % 在 q0 初始位置和终止位置添加圆点标记
    scatter3(q0(1), q0(2), q0(3), 100, 'r', 'filled');  % 初始位置
    scatter3(q0_trajectory_values(1, end), q0_trajectory_values(2, end), q0_trajectory_values(3, end), 100, 'g', 'filled');  % 终止位置

    % 设置图形比例，确保球体是圆形的
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Trajectory of q(t) in 3D space with Safety Zone');
    legend('q(t) trajectory', 'Safety Zone (r = 1)', 'Initial Position', 'Final Position');

    % 在图中添加标注框，标明初始和终止位置
    z_offset = 0.2;  % 添加一个偏移量，确保标注框不与轨迹重叠
    text(q0(1), q0(2), q0(3) + z_offset, sprintf('  q0 t=0s\n[%.4f, %.4f, %.4f]', initial_q0(1), initial_q0(2), initial_q0(3)), 'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
    text(q0_trajectory_values(1, end), q0_trajectory_values(2, end), q0_trajectory_values(3, end) + z_offset, sprintf('  q0 t=10s\n[%.4f, %.4f, %.4f]', final_q0(1), final_q0(2), final_q0(3)), 'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    
    % 设置 z 轴范围，确保球体完全显示
    zlim([-1, 3.5]);  % 设置 z 轴的显示范围为 [-1, 5]，以便看到球体的上半部分

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
