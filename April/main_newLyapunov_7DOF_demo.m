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
    beta_hat0 = ones(n,1) * 1;       % 初始估计beta_hat

    % 状态 X = [q; dq; beta_hat]
    X = [x0_q; x0_dq; beta_hat0];

    % 用于记录
    X_hist        = zeros(3*n, length(tspan));
    beta_hat_hist = zeros(n,   length(tspan));
    V_new_hist    = zeros(1,   length(tspan));
    dotV_new_hist = zeros(1,   length(tspan));

    % 参考与 PD 设定
    q_d  = zeros(n,1);
    dq_d = zeros(n,1);
    Kp = 10 * ones(n,1);
    Kd = 5  * ones(n,1);

    %% ================ 5. 主循环 (Euler) ==================
    for i = 1:length(tspan)
        % 拆分状态
        q        = X(1:n);
        dq       = X(n+1:2*n);
        beta_hat = X(2*n+1:3*n);
        beta_hat_hist(:,i) = beta_hat;

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

        % 扩展 Lyapunov，不使用 beta_true
        V_new = V + 0.5 * (beta_hat.^2);
        V_new_hist(i) = sum(V_new);

        % 梯度 L1 用于自适应律
        gradV_L1 = abs(e) + abs(de);

        % 自适应律：beta_hat 自增长
        dot_beta_hat = gradV_L1 .* sigma_vec;

        % Lyapunov 导数
        ddq_total = f2 + u ./ diag(M_mat);
        dotV = e .* dq + de .* ddq_total;
        dotV_new = sum(dotV + beta_hat .* dot_beta_hat);
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
    figure('Position',[100,100,1200,800]);

    subplot(2,1,1);
    plot(tspan, beta_hat_hist', 'LineWidth',1.2);
    xlabel('t [s]'); ylabel('beta\_hat');
    title('各关节自适应beta\_hat');
    legend(arrayfun(@(j) sprintf('joint%d',j),1:n,'Uni',0), 'Location','eastoutside');
    grid on;

    subplot(2,1,2);
    plot(tspan, V_new_hist, 'LineWidth',1.5);
    hold on;
    plot(tspan, dotV_new_hist, '--', 'LineWidth',1.5);
    xlabel('t [s]'); ylabel('Value');
    title('V\_new (实线) 与 dotV\_new (虚线)');
    legend('V\_new','dotV\_new');
    grid on;

    disp('=== 7DOF 自适应仿真结束 ===');
end