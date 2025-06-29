function main_newLyapunov_demo()
    clear; clc; close all;

    % ================ 1. 定义系统与 GP 模型相关参数 ==================

    % 机械系统参数
    M = 1.0;          % 质量
    k = 0.1;          % C(dq) = k * dq^2
    m = 1.0;          % 用于重力项
    g = 9.8;          % 重力加速度
    C = @(dq) k * dq^2;         % 科氏/离心力项
    g_func = @(q) m * g * sin(q);% 重力项
    d_true = @(x) sin(x(1)) + cos(x(2));  % 未知动态(真实)

    % 时间设置
    tspan = 0:0.01:10;    % 仿真 0~10s，步长0.01
    dt = tspan(2) - tspan(1);

    % 初始状态 [q(0), dq(0)]
    x0 = [1; 1];

    % ================ 2. GP模型 & 超参数设置 =====================
    %   (这部分和你原代码类似，只是示例 LocalGP_MultiOutput)
    gp = LocalGP_MultiOutput(2, 1, 200, 0.1, 0.5, 1.5);
    %   这里的  (dim_in=2, dim_out=1, MaxDataQuantity=200, noise=0.1, SigmaF=0.5, SigmaL=1.5)
    %   你可根据需要修改

    % “真实超参数” 和 “名义超参数” 不再细分，这里只要
    %   gp 内部是 (SigmaF=0.5, SigmaL=1.5)，就视为 真·theta_true
    %   你若想要对比 name/true，可像以前那样切换

    % 初始化 GP 数据集: 先收集一些点,免得一开始没数据 sigma=0
    initX = [0, 0;  1, 1;  -1,0;  0,2;  2,2]';  % 5个点，纯演示
    for ii=1:size(initX,2)
        gp.addPoint(initX(:,ii), d_true(initX(:,ii)));
    end
    disp(['[Init] GP DataQuantity = ', num2str(gp.DataQuantity)]);

    % ================ 3. 定义 新的 beta(t) 及其相关参数 ================
    beta_true = 1.0;         % 真值
    beta_hat0 = 1.5;         % 初值
    % dot_beta_hat = (...), 稍后会在 ODE/循环里根据公式做更新
    % beta_tilde = beta_hat - beta_true

    % ================ 4. 主仿真循环 (简单Euler 或 ODE45都可以) ================

    % 为演示方便，这里手工 for-loop + Euler 更新
    %   状态扩展为 X = [ q; dq; beta_hat ]  (3维)
    X = [x0; beta_hat0];  % 初始
    X_history = zeros(3, length(tspan));
    V_new_history = zeros(1, length(tspan));
    dotV_new_history = zeros(1, length(tspan));

    % 你若有参考轨迹 q_d(t)，可定义；此处简单令 q_d=0, dq_d=0
    q_d = 0;  dq_d = 0;
    Kp = 10;  Kd = 5;  % PD控制增益 (可自行调)

    for i = 1:length(tspan)
        t = tspan(i);
        q  = X(1);
        dq = X(2);
        beta_hat = X(3);

        % -------------- (a) GP 预测均值 & 方差 ----------------
        current_x = [q; dq];
        [mu_val, sigma_val, ~, ~, ~, ~, ~] = gp.predict(current_x);
        % sigma_val 这里是 GP 给出的预测“方差”还是“协方差”，
        %   如果你想用标准差，可 sqrt(sigma_val)；看你论文中怎么定义。

        % -------------- (b) 系统动力学 f(x) ---------------
        %   真·系统的加速度 ddq = ( d_true(x) - C(dq)*dq - g(q) ) / M
        %   这里我们记 f(x) = [  dq;
        %                          (d_true - C*dq - g(q))/M  ]   （即 xdot 不含控制u）
        f_val_1 = dq;
        f_val_2 = ( d_true(current_x) - C(dq)*dq - g_func(q) ) / M;

        % -------------- (c) 控制器 u ---------------
        %   目标: 令 q -> q_d(=0), dq -> dq_d(=0)
        e  = q - q_d;
        de = dq - dq_d;   % 这里记 s = de
        %   简单PD:
        u = -Kp*e - Kd*de;  % scalar

        % -------------- (d) 新增 Lyapunov 计算 ---------------
        %   1) 定义 V(x) = 0.5*e^2 + 0.5*de^2
        %   2) beta_tilde = beta_hat - beta_true
        %   3) V_new = V + 0.5*(beta_tilde)^2
        V    = 0.5*(e^2 + de^2);
        btil = beta_hat - beta_true;
        V_new = V + 0.5*(btil^2);

        %   4) 计算 dV/dx
        %   x = [q; dq],  e = q - q_d => ∂e/∂q=1, ∂e/∂dq=0
        %                  de= dq - dq_d => ∂de/∂q=0, ∂de/∂dq=1
        %   V = 0.5 e^2 + 0.5 de^2 => dV/d(e)= e, dV/d(de)= de
        %   => dV/dq = e, dV/ddq = de
        gradV_wrt_x = [ e;  de ];  % 2×1
        %   L1范数(一范数): ||gradV||_1 = |e| + |de|
        gradV_L1 = abs(e) + abs(de);

        %   5) dot_beta_hat = (||dV/dx||_1)* sigma_val
        %       (假设用 sigma_val 直接做因子，视需求可 sqrt)
        dot_beta_hat = gradV_L1 * sigma_val;

        %   6) dot_V = gradV_wrt_x' * ( f(x) + u )，实际上对 x=[q; dq]：
        %       dot_V = e*(dq) + de*(ddq)  (其中 ddq= f_val_2 + u/M ？要注意别漏掉u对加速度的贡献)
        %     但这里根据你给的公式:
        %       dot_V_new = [∂V/∂x]*[ mu + u ] + [∂V/∂x]*[ f(x) - mu ] + btil * dot_beta_hat
        %                  = [∂V/∂x]*[ f(x) + u ] + btil * dot_beta_hat
        %     这里为了简洁，咱们就直接用“真实 f + u”来算即可:
        ddq_total = f_val_2 + u / M;
        dot_V = e * dq + de * ddq_total;  % a·b = e*dq + de*ddq

        dot_V_new = dot_V + btil * dot_beta_hat;

        % -------------- (e) 整合成 ODE，做离散更新 ---------------
        %   xdot(1) = dq
        %   xdot(2) = ddq_total = f_val_2 + u/M
        %   xdot(3) = dot_beta_hat
        xdot = [ dq;
                  f_val_2 + (u / M);
                  dot_beta_hat ];

        % Euler积分 (可换成 ode45 亦可)
        X = X + xdot * dt;

        % 记录
        X_history(:,i) = X; 
        V_new_history(i) = V_new;
        dotV_new_history(i) = dot_V_new;
    end

    % ================ 5. 画图查看收敛性 =====================

    figure('Name','States');
    subplot(3,1,1);
    plot(tspan, X_history(1,:), 'LineWidth',1.5);
    xlabel('t'); ylabel('q(t)');
    title('关节角 q(t)');

    subplot(3,1,2);
    plot(tspan, X_history(2,:), 'LineWidth',1.5);
    xlabel('t'); ylabel('dq(t)');
    title('角速度 dq(t)');

    subplot(3,1,3);
    plot(tspan, X_history(3,:), 'LineWidth',1.5);
    xlabel('t'); ylabel('\beta_{hat}(t)');
    title('额外参数 \beta_{hat}(t)');

    figure('Name','Lyapunov V_{new}');
    subplot(2,1,1);
    plot(tspan, V_new_history, 'LineWidth',1.5);
    xlabel('t'); ylabel('V_{new}');
    title('V_{new}(t) = V(x) + 0.5(\beta_{tilde})^2');

    subplot(2,1,2);
    plot(tspan, dotV_new_history, 'LineWidth',1.5);
    xlabel('t'); ylabel('\dot{V}_{new}');
    title('李雅普诺夫函数导数 \dot{V}_{new}(t)');

    disp('===== 仿真结束 =====');
    disp(['最终 \beta_{hat} = ', num2str(X_history(3,end)), ...
          ', 其真值 \beta_{true} = ', num2str(beta_true)]);
end


%% ============= 下面给出一个最简版本 LocalGP_MultiOutput 类(示例) ===============
% 仅保证能运行, 实际功能很简陋, 你可用自己那份更完整的GP类来替换
classdef LocalGP_MultiOutput < handle
    properties
        dim_in
        dim_out
        MaxDataQuantity
        noise
        SigmaF
        SigmaL

        DataX   % (dim_in, N)
        DataY   % (dim_out, N)
        DataQuantity
    end

    methods
        function obj = LocalGP_MultiOutput(dim_in, dim_out, MaxDataQuantity, noise, SigmaF, SigmaL)
            obj.dim_in = dim_in;
            obj.dim_out = dim_out;
            obj.MaxDataQuantity = MaxDataQuantity;
            obj.noise = noise;
            obj.SigmaF = SigmaF;
            obj.SigmaL = SigmaL;
            obj.DataX = [];
            obj.DataY = [];
            obj.DataQuantity = 0;
        end

        function addPoint(obj, x, y)
            if obj.DataQuantity < obj.MaxDataQuantity
                obj.DataX = [obj.DataX, x];
                obj.DataY = [obj.DataY, y];
                obj.DataQuantity = obj.DataQuantity + 1;
            else
                % 简单做法：若满了就丢掉最早的一条 (FIFO)
                obj.DataX(:,1) = [];
                obj.DataY(:,1) = [];
                obj.DataX = [obj.DataX, x];
                obj.DataY = [obj.DataY, y];
            end
        end

        function [mu, sigma, ~, ~, ~, ~, ~] = predict(obj, x_star)
            % 一个非常简陋的GP预测，仅做示例
            %   真实场景要计算核矩阵并带上噪声等
            %   这里“伪造”一个与距离有关的插值
            if obj.DataQuantity == 0
                mu = 0;   sigma = 1;  % 无数据时
                return;
            end
            % 构造核函数
            k = @(xa,xb) obj.SigmaF * exp(-0.5*sum((xa - xb).^2)/(obj.SigmaL^2));

            % 计算 K(X,X)+noiseI, k(X,x_star)
            K_mat = zeros(obj.DataQuantity, obj.DataQuantity);
            for i=1:obj.DataQuantity
                for j=1:obj.DataQuantity
                    K_mat(i,j) = k(obj.DataX(:,i), obj.DataX(:,j));
                end
            end
            K_mat = K_mat + obj.noise*eye(obj.DataQuantity);

            k_star = zeros(obj.DataQuantity,1);
            for i=1:obj.DataQuantity
                k_star(i) = k(obj.DataX(:,i), x_star);
            end

            % 均值 (K(X,X)+nI)^-1 * k(X,x_star)
            alpha = K_mat \ k_star;   % size Nx1
            mu = (obj.DataY * alpha)';% out为1×1

            % 方差 sigma = k(x_star,x_star) - k_star^T * invK * k_star
            k_xx = k(x_star, x_star);
            sigma = k_xx - k_star' * alpha;
            if sigma<1e-10
                sigma=1e-10; % 防止负数或过小
            end
        end
    end
end
