function main_newLyapunov_7DOF_demo()
    clear; clc; close all;

    %% ================ 1. 系统与 GP 参数 ==================
    n = 7;                              % 自由度数
    M_mat = eye(n);                     % 质量矩阵（对角）
    k     = 0.1;                        % 科氏/离心力系数
    m     = ones(n,1);                  % 各关节质量
    g     = 9.8;                        % 重力加速度

    C     = @(dq) k .* (dq.^2);                     % 元素级 Coriolis
    g_fun = @(q) m .* g .* sin(q);                  % 元素级重力项
    d_true = @(x) sin(x(1:n)) + cos(x(n+1:2*n));    % 未知动力学：7×1

    % 时间设置
    tspan = 0:0.01:10;
    dt    = tspan(2) - tspan(1);

    %% ================ 2. 初始状态 =====================
    x0_q  = [2.0; 1.5; 1.0; 0.5; -1.0; -0.5; 0.2];  % 各关节初始角度
    x0_dq = [0.0; 0.2; -0.1; 0.3; -0.2; 0.1; 0.0];  % 各关节初始速度

    %% ================ 3. GP 模型初始化 ==================
    input_dim  = 2*n;   % q 和 dq 串联
    output_dim = n;     % 每个关节一个输出
    gp = LocalGP_MultiOutput(input_dim, output_dim, 500, 0.1, 0.5, 1.5);

    % 预采样初始化，防止 sigma=0
    initX = [randn(n,50); randn(n,50)];
    for ii = 1:size(initX,2)
        gp.addPoint(initX(:,ii), d_true(initX(:,ii)));
    end
    fprintf('[Init] GP DataQuantity = %d\n', gp.DataQuantity);

    %% ================ 4. 扩展状态初始化 ==================
    beta_true = 1.9;                   % 真值
    beta_hat0 = ones(n,1) * 1.0;       % 初始估计

    % 状态 X = [q; dq; beta_hat]
    X = [x0_q; x0_dq; beta_hat0];

    % 用于记录
    X_hist        = zeros(3*n, length(tspan));
    delta_tilde   = zeros(n,   length(tspan));
    V_new_hist    = zeros(1,   length(tspan));
    dotV_new_hist = zeros(1,   length(tspan));

    % 安全/参考轨迹
    p0   = zeros(n,1);
    r    = 1.5;
    q_d  = p0;
    dq_d = zeros(n,1);

    % PD 增益（保持相同）
    Kp = 10 * ones(n,1);
    Kd = 5  * ones(n,1);

    %% ================ 5. 主循环 (Euler) ==================
    for i = 1:length(tspan)
        % 拆分状态
        q        = X(1:n);
        dq       = X(n+1:2*n);
        beta_hat = X(2*n+1:3*n);

        % 估计误差
        btil = beta_hat - beta_true;
        delta_tilde(:,i) = btil;

        % GP 预测
        x_gp = [q; dq];
        [mu_vec, sigma_vec, ~, ~, ~, ~, ~] = gp.predict(x_gp);

        % 系统动力学
        f1 = dq;
        f2 = (d_true(x_gp) - C(dq).*dq - g_fun(q)) ./ diag(M_mat);

        % PD 控制
        e  = q - q_d;
        de = dq - dq_d;
        u  = -Kp .* e - Kd .* de;

        % Lyapunov 基本量
        V = 0.5 * (e.^2 + de.^2);

        % 扩展 Lyapunov
        V_new = V + 0.5 * (btil.^2);
        V_new_hist(i) = sum(V_new);

        % 梯度 L1
        gradV_L1 = abs(e) + abs(de);

        % 自适应律
        dot_beta_hat = gradV_L1 .* sigma_vec;

        % V_dot
        ddq_total = f2 + u ./ diag(M_mat);
        dotV = e .* dq + de .* ddq_total;
        dotV_new = sum(dotV + btil .* dot_beta_hat);
        dotV_new_hist(i) = dotV_new;

        % 状态更新
        xdot = [f1; ddq_total; dot_beta_hat];
        X = X + dt * xdot;

        % GP 数据管理
        if gp.DataQuantity >= gp.MaxDataQuantity
            gp.downdateParam(1);
        end
        gp.addPoint(x_gp, d_true(x_gp));

        X_hist(:,i) = X;
    end

    %% ================ 6. 绘图 ==================
    figure('Position', [100, 100, 1200, 800], 'Renderer', 'painters');

% 创建缩放后的时间向量（缩小一倍）
tspan_scaled = tspan / 2;

%----------- 子图1：beta估计误差 ------------
subplot(3,1,1);
plot(tspan_scaled, delta_tilde', 'LineWidth', 1.5);  % 使用缩放后的时间
xlabel('$t$ [s]', 'Interpreter', 'latex'); 
ylabel('$\hat{\beta} - \beta_{\mathrm{true}}$', 'Interpreter', 'latex');
title('Parameter Estimation Error $\tilde{\beta}(t)$ for All Joints', 'Interpreter', 'latex');
legend(arrayfun(@(j) sprintf('Joint %d', j), 1:n, 'UniformOutput', false), ...
      'Location', 'eastoutside', 'Interpreter', 'latex');
grid on;
box on;

%----------- 子图2：新型Lyapunov函数 ------------
subplot(3,1,2);
plot(tspan_scaled, V_new_hist, 'LineWidth', 1.5);  % 使用缩放后的时间
xlabel('$t$ [s]', 'Interpreter', 'latex'); 
ylabel('$V_{\mathrm{new}}(t)$', 'Interpreter', 'latex');
title('Lyapunov Function $V_{\mathrm{new}}(t) = V(x) + \frac{1}{2}\tilde{\beta}^{\top}\Gamma^{-1}\tilde{\beta}$', ...
      'Interpreter', 'latex');
grid on;
box on;

%----------- 子图3：Lyapunov导数 ------------
subplot(3,1,3);
plot(tspan_scaled, dotV_new_hist, 'LineWidth', 1.5);  % 使用缩放后的时间
xlabel('$t$ [s]', 'Interpreter', 'latex'); 
ylabel('$\dot{V}_{\mathrm{new}}(t)$', 'Interpreter', 'latex');
title('Time Derivative of Lyapunov Function $\dot{V}_{\mathrm{new}}(t)$', ...
      'Interpreter', 'latex');
grid on;
box on;

% 确保立即渲染
drawnow;

    disp('=== 7DOF 仿真结束 ===');
end
