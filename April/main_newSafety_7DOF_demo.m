function main_newSafety_7DOF_demo()
    %% ================ 1. 定义系统与 GP 模型相关参数 ==================
    clear; clc; close all;
    
    % 自由度
    n = 7;
    
    % 机械臂参数
    M_mat = eye(n);      % 简化为对角质量矩阵
    k     = 0.1;         % 科氏/离心力系数
    m     = ones(n,1);   % 各关节质量
    g     = 9.8;         % 重力加速度
    
    C     = @(dq) k .* (dq.^2);
    g_fun = @(q) m .* g .* sin(q);
    d_true = @(x) sin(x(1:n)) + cos(x(n+1:2*n));  % 未知动力学
    
    % 仿真时间
    tspan = 0:0.01:10;
    dt    = tspan(2) - tspan(1);
    
    %% ================ 2. 初始状态 =====================
    x0_q  = [2.0; 1.5; 1.0; 0.5; -1.0; -0.5; 0.2];
    x0_dq = [0.0; 0.2; -0.1; 0.3; -0.2; 0.1; 0.0];
    
    %% ================ 3. GP 模型初始化 ==================
    input_dim  = 2*n;
    output_dim = n;
    gp = LocalGP_MultiOutput(input_dim, output_dim, 500, 0.1, 0.5, 1.5);
    initX = [randn(n,50); randn(n,50)];
    for ii = 1:size(initX,2)
        gp.addPoint(initX(:,ii), d_true(initX(:,ii)));
    end
    fprintf('[Init] GP DataQuantity = %d\n', gp.DataQuantity);
    
    %% ================ 4. 扩展状态初始化 ==================
    % 不再使用未知的 beta_true
    beta_hat0 = 1 * ones(n,1);  % 初始 beta_hat
    X = [x0_q; x0_dq; beta_hat0];
    
    % 数据记录
    delta_tilde = [];  % 可删除或用于记录 beta_hat
    h_new_hist   = zeros(1,length(tspan));
    doth_new_hist= zeros(1,length(tspan));
    beta_hat_hist = zeros(n,length(tspan));
    
    q_d  = zeros(n,1);
    dq_d = zeros(n,1);
    Kp = 10 * ones(n,1);
    Kd = 5  * ones(n,1);
    
    %% ================ 5. 主循环 (Euler) ==================
    for i = 1:length(tspan)
        q        = X(1:n);
        dq       = X(n+1:2*n);
        beta_hat = X(2*n+1:3*n);
        beta_hat_hist(:,i) = beta_hat;
        
        % GP 预测
        x_gp = [q; dq];
        [~, sigma_vec, ~, ~, ~, ~, ~] = gp.predict(x_gp);
        
        % 系统动力学
        f1 = dq;
        f2 = (d_true(x_gp) - C(dq).*dq - g_fun(q)) ./ diag(M_mat);
        
        % PD 控制
        e  = q - q_d;
        de = dq - dq_d;
        u  = -Kp .* e - Kd .* de;
        
        % 安全函数 (可选)
        h      = 1.5^2 - sum((q - zeros(n,1)).^2);
        h_new  = h + 0.5*sum(beta_hat.^2);
        h_new_hist(i) = h_new;
        
        % 自适应律
        grad_h_L1    = 2 * abs(q);  % 例：使用 q 的偏导范数
        dot_beta_hat = grad_h_L1 .* sigma_vec;
        
        % 安全函数导数 (可选)
        h_dot       = -2*(q')*dq;
        doth_new    = h_dot + sum(beta_hat .* dot_beta_hat);
        doth_new_hist(i) = doth_new;
        
        % 状态更新
        ddq_total = f2 + u ./ diag(M_mat);
        xdot = [f1; ddq_total; dot_beta_hat];
        X = X + dt * xdot;
        
        % GP 数据管理
        if gp.DataQuantity >= gp.MaxDataQuantity
            gp.downdateParam(1);
        end
        gp.addPoint(x_gp, d_true(x_gp));
    end
    
    %% ================ 6. 绘图 ==================
    figure('Position',[100,100,1200,800]);
    subplot(2,1,1);
    plot(tspan, beta_hat_hist','LineWidth',1.2);
    xlabel('t [s]'); ylabel('beta\_hat');
    title('各关节自适应 beta\_hat');
    legend(arrayfun(@(j) sprintf('joint%d',j),1:n,'Uni',0),'Location','eastoutside');
    grid on;
    
    subplot(2,1,2);
    plot(tspan, h_new_hist,'LineWidth',1.5);
    hold on;
    plot(tspan, doth_new_hist,'--','LineWidth',1.5);
    xlabel('t [s]'); ylabel('Value');
    title('h\_new & dot h\_new');
    legend('h\_new','dot h\_new');
    grid on;
    
    disp('=== 7DOF 自适应仿真结束（无 beta\_true） ===');
end
