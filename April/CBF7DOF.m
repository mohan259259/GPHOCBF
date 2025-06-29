function main_newLyapunov_7DOF_demo()
    %% ================ 1. 定义系统与 GP 模型相关参数 ==================
    clear; clc; close all;
    
    % 自由度
    n = 7;
    
    % 机械臂参数（简化为对角质量矩阵，重力、科氏力逐关节独立处理）
    M_mat = eye(n);
    k     = 0.1;
    m     = ones(n,1);
    g     = 9.8;
    
    C     = @(dq) k .* (dq.^2);
    g_fun = @(q) m .* g .* sin(q);
    d_true = @(x) sin(x(1:n)) + cos(x(n+1:2*n));
    
    % 仿真时间
    tspan = 0:0.01:10;
    dt    = tspan(2) - tspan(1);
    
    %% ================ 2. 不同初始条件设置 =====================
    % 每个关节不同初始角度和速度
    x0_q  = [2.0; 1.5; 1.0; 0.5; -1.0; -0.5; 0.2];
    x0_dq = [0.0; 0.2; -0.1; 0.3; -0.2; 0.1; 0.0];

    %% ================ 3. GP模型 & 超参数设置 =====================
    input_dim   = 2*n;
    output_dim  = n;
    MaxDataQ    = 500;
    SigmaN      = 0.1;
    SigmaF      = 0.5;
    SigmaL      = 1.5;
    
    gp = LocalGP_MultiOutput(input_dim, output_dim, ...
                             MaxDataQ, SigmaN, SigmaF, SigmaL);
    
    % 初始化 GP 数据集
    initX = [ randn(n,50); randn(n,50) ];
    for ii = 1:size(initX,2)
        gp.addPoint(initX(:,ii), d_true(initX(:,ii)));
    end
    fprintf('[Init] GP DataQuantity = %d\n', gp.DataQuantity);
    
    %% ================ 4. 扩展状态初始化 =========================
    beta_true   = 0.5;
    beta_hat0   = 1.5 * ones(n,1);
    
    X            = [x0_q; x0_dq; beta_hat0];
    X_history    = zeros(3*n, length(tspan));
    delta_tilde  = zeros(n,   length(tspan));
    h_new_hist   = zeros(1,   length(tspan));
    doth_new_hist= zeros(1,   length(tspan));
    
    p0   = zeros(n,1);
    r    = 1.5;
    
    q_d  = p0;
    dq_d = zeros(n,1);
    
    % PD 增益：Kp 相同，Kd 相同
    Kp = 10 * ones(n,1);
    Kd = 5  * ones(n,1);
    
    %% ================ 5. 主仿真循环 (Euler 法) ==================
    for i = 1:length(tspan)
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
        f2 = ( d_true(x_gp) - C(dq).*dq - g_fun(q) ) ./ diag(M_mat);
        
        % PD 控制
        e  = q  - q_d;
        de = dq - dq_d;
        u  = -Kp .* e - Kd .* de;
        
        % 安全函数
        h      = r^2 - sum((q - p0).^2);
        h_new  = h + 0.5 * sum(btil.^2);
        h_new_hist(i) = h_new;
        
        grad_h       = -2 * (q - p0);
        grad_h_L1    =  2 * abs(q - p0);
        dot_beta_hat = grad_h_L1 .* sigma_vec;
        
        % h_new 导数
        h_dot    = grad_h' * dq;
        doth_new = h_dot + btil' * dot_beta_hat;
        doth_new_hist(i) = doth_new;
        
        % 状态更新 (Euler)
        ddq_total = f2 + u ./ diag(M_mat);
        xdot = [ f1;
                 ddq_total;
                 dot_beta_hat ];
        X    = X + dt * xdot;
        
        % 数据管理
        if gp.DataQuantity >= gp.MaxDataQuantity
            gp.downdateParam(1);
        end
        gp.addPoint(x_gp, d_true(x_gp));
        
        X_history(:,i) = X;
    end
    
    %% ================ 6. 绘图 ===================
    figure('Position',[100 100 1200 800]);
    
    subplot(3,1,1);
    plot(tspan, delta_tilde', 'LineWidth',1.2);
    xlabel('t [s]'); ylabel('\betâ - \beta');
    title('beta esti for every joints');
    legend(arrayfun(@(j) sprintf('joint %d',j),1:n,'Uni',0),...
           'Location','eastoutside');
    grid on;
    
    subplot(3,1,2);
    plot(tspan, h_new_hist, 'LineWidth',1.5);
    xlabel('t [s]'); ylabel('h{new}');
    title(' h function');
    grid on;
    
    subplot(3,1,3);
    plot(tspan, doth_new_hist, 'LineWidth',1.5);
    xlabel('t [s]'); ylabel('\dot h{new}');
    title('dot h');
    grid on;
    
    disp('=== 7DOF 仿真结束 ===');
end
