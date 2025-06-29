%% MATLAB 脚本：预设时间安全控制律验证 (CBF 方法)
clear; clc; close all;

%% 参数定义
% 控制相关参数
alpha0 = 1;             % alpha0参数
alpha1 = 1;             % alpha1参数
T_s = 2;                % 预设安全时间
T_theta = 1;            % 预设参数收敛时间
t0_val = 0;             % 初始时间 (用于计算 blow-up 函数)
c_gain = 1;             % 安全控制裕度增益 (GP不确定度系数)

% 二维机械臂物理参数 (假设链接长度和质量均为1，考虑重力)
L1 = 1; L2 = 1;         % 连杆长度 [m]
m1 = 1; m2 = 1;         % 连杆质量 [kg]
I1 = 1/3 * m1 * L1^2;   % 连杆1转动惯量 (假设均匀杆) 
I2 = 1/3 * m2 * L2^2;   % 连杆2转动惯量
g_const = 9.81;         % 重力加速度 [m/s^2]

% 安全区域参数
p0 = [0.5; 0.5];        % 安全球区域中心 (工作空间坐标)
r = 0.2;                % 安全球半径

% GP预测参数 (针对未建模动态的不确定项)
mu_hat = [1; 1];        % GP预测的未知力矩均值 (每个关节)
sigma_hat = [0.5; 0.5]; % GP预测的不确定性标准差

% PD控制器增益
Kp = 50 * eye(2);       % 比例增益矩阵
Kd = 15 * eye(2);       % 微分增益矩阵

% 目标末端位置对应的关节角 (安全区中心附近)
q_des = [1.997; -2.4189];   % 目标关节角 [rad]，使末端位于 p0 附近
qd_des = [0; 0];            % 目标关节角速度 [rad/s] (静止)

% 初始状态 (设末端在安全区域边界外)
q0 = [1.7972; -2.0236];     % 初始关节角 [rad]
qd0 = [0; 0];               % 初始关节角速度 [rad/s]
x0 = [q0; qd0];             % 初始状态向量

% 将参数打包进结构体
params.m1 = m1; params.m2 = m2; params.L1 = L1; params.L2 = L2;
params.I1 = I1; params.I2 = I2; params.g_const = g_const;
params.p0 = p0; params.r = r;
params.mu_hat = mu_hat; params.sigma_hat = sigma_hat;
params.Kp = Kp; params.Kd = Kd;
params.q_des = q_des; params.qd_des = qd_des;
params.alpha0 = alpha0; params.alpha1 = alpha1;
params.T_s = T_s; params.T_theta = T_theta;
params.t0_val = t0_val; params.c_gain = c_gain;

%% 数值仿真 (积分动力学方程)
ode_func = @(t,x) manipulator_dynamics(t, x, params);
tspan = [0, 3];                         % 仿真时间从0到3秒 (覆盖 T_s 之后)
opts = odeset('RelTol',1e-6, 'MaxStep',1e-3);
[t_out, x_out] = ode45(ode_func, tspan, x0, opts);

%% 计算 h(q(t)) 随时间的变化
q_out = x_out(:, 1:2);
num_pts = length(t_out);
h_vals = zeros(num_pts, 1);
for i = 1:num_pts
    % 末端位置坐标
    q = q_out(i, :)'; 
    x_end = L1 * cos(q(1)) + L2 * cos(q(1) + q(2));
    y_end = L1 * sin(q(1)) + L2 * sin(q(1) + q(2));
    pos = [x_end; y_end];
    % 安全函数 h(q) = r^2 - ||pos - p0||^2
    h_vals(i) = r^2 - sum((pos - p0).^2);
end

%% 绘制 h(q(t)) 随时间变化曲线
figure;
plot(t_out, h_vals, 'LineWidth', 2); hold on;
yline(0, '--r', 'LineWidth', 1);           % h=0 安全边界
xline(T_s, '--k', 'LineWidth', 1);         % 预设安全时间 T_s
xlabel('时间 t (秒)'); ylabel('h(q(t))');
title('安全函数 h(q) 随时间的变化');
legend({'h(q(t))','安全边界 h=0','T_s'}, 'Location','best');
grid on;

%% 验证结果
min_h = min(h_vals(t_out >= T_s));  % 计算 t >= T_s 之后的最小 h
disp(['Ts 后最小 h 值 = ', num2str(min_h)]);
if min_h > 0
    disp('系统在 T_s 之后保持安全 (h(q) 始终为正)。');
else
    disp('系统未能在 T_s 之后保持安全，存在 h(q) <= 0 的情况！');
end

%% 动力学和控制算法的函数定义
function dx = manipulator_dynamics(t, x, params)
    % 提取状态
    q = x(1:2);             % 关节角 [q1; q2]
    qd = x(3:4);            % 关节角速度 [q1_dot; q2_dot]
    % 惯性矩阵 M(q)
    M11 = params.m1*params.L1^2 + params.m2*(params.L1^2 + 2*params.L1*params.L2*cos(q(2)) + params.L2^2) + params.I1 + params.I2;
    M12 = params.m2*(params.L1*params.L2*cos(q(2)) + params.L2^2) + params.I2;
    M21 = M12;
    M22 = params.m2*params.L2^2 + params.I2;
    M = [M11, M12; M21, M22];
    % 科里奥利离心项 C(q, qd)*qd
    C11 = -params.m2*params.L1*params.L2*sin(q(2)) * qd(2);
    C12 = -params.m2*params.L1*params.L2*sin(q(2)) * (qd(1) + qd(2));
    C21 =  params.m2*params.L1*params.L2*sin(q(2)) * qd(1);
    C22 = 0;
    C_mat = [C11, C12; C21, C22];
    Cqd = C_mat * qd;
    % 重力项 g(q)
    g1 = (params.m1*params.L1/2 + params.m2*params.L1) * params.g_const * cos(q(1)) ...
       + params.m2 * params.L2/2 * params.g_const * cos(q(1) + q(2));
    g2 = params.m2 * params.L2/2 * params.g_const * cos(q(1) + q(2));
    g_vec = [g1; g2];
    % 未建模扰动 (实际值，用于仿真)
    d_actual = [1.5; 1.5];   % 假设每个关节有恒定1.5 N·m扰动

    % 计算安全函数及其导数需要的梯度和Hessian
    % 末端位置及 h(q)
    x_end = params.L1 * cos(q(1)) + params.L2 * cos(q(1) + q(2));
    y_end = params.L1 * sin(q(1)) + params.L2 * sin(q(1) + q(2));
    pos = [x_end; y_end];
    h_val = params.r^2 - sum((pos - params.p0).^2);
    % h 对 q 的梯度 ∇h(q)
    J11 = -params.L1*sin(q(1)) - params.L2*sin(q(1)+q(2));
    J12 = -params.L2*sin(q(1)+q(2));
    J21 =  params.L1*cos(q(1)) + params.L2*cos(q(1)+q(2));
    J22 =  params.L2*cos(q(1)+q(2));
    J = [J11, J12; J21, J22];
    delta_p = pos - params.p0;
    grad_h_pos = -2 * delta_p.';             % ∂h/∂p (1x2)
    grad_h = grad_h_pos * J;                 % ∇_q h (1x2 行向量)
    % 近似计算 qd^T * ∇^2h * qd (有限差分)
    dq = 1e-6;
    h_plus1 = compute_h(q + [dq; 0], params);
    h_minus1 = compute_h(q - [dq; 0], params);
    h_plus2 = compute_h(q + [0; dq], params);
    h_minus2 = compute_h(q - [0; dq], params);
    d2h_dq1q1 = (h_plus1 - 2*h_val + h_minus1) / (dq^2);
    d2h_dq2q2 = (h_plus2 - 2*h_val + h_minus2) / (dq^2);
    h_pp = compute_h(q + [dq; dq], params);
    h_pm = compute_h(q + [dq; -dq], params);
    h_mp = compute_h(q + [-dq; dq], params);
    h_mm = compute_h(q + [-dq; -dq], params);
    d2h_dq1q2 = (h_pp - h_pm - h_mp + h_mm) / (4*dq^2);
    d2h_dq2q1 = d2h_dq1q2;
    Hessian_h = [d2h_dq1q1, d2h_dq1q2; d2h_dq2q1, d2h_dq2q2];
    qdHqd = qd.' * Hessian_h * qd;
    % 名义控制 (PD控制 + 重力补偿 + 未知均值补偿)
    tau_pd = -params.Kp * (q - params.q_des) - params.Kd * (qd - params.qd_des);
    tau_nom = tau_pd + g_vec + params.mu_hat;   % tau_n
    % 安全控制校正 (CBF约束)
    invM = M \ eye(2);
    L_vec = grad_h * invM;   % L = ∇h * M^{-1} (1x2)
    known_vec = Cqd + g_vec + params.mu_hat;
    known_contrib = grad_h * (M \ known_vec);
    % 时间相关增益 z0(t), z1(t)
    if t < params.T_theta
        z1 = params.alpha1 / (params.T_theta - (t - params.t0_val));
    else
        z1 = params.alpha1;
    end
    if t < params.T_s
        z0 = params.alpha0 / (params.T_s - (t - params.t0_val));
    else
        z0 = params.alpha0;
    end
    % 不等式右端 S 值计算
    S_val = known_contrib - (qdHqd + (z0 + z1)* (grad_h * qd).' + z0*z1 * h_val);
    % 不确定性裕度 (基于 sigma_hat)
    margin = params.c_gain * (abs(L_vec(1))*params.sigma_hat(1) + abs(L_vec(2))*params.sigma_hat(2));
    S_val = S_val - margin;
    L_tau_n = L_vec * tau_nom;
    if L_tau_n < S_val
        if norm(L_vec) < 1e-6
            tau_cbf = [0; 0];
        else
            k_val = (S_val - L_tau_n) / (L_vec * L_vec.');
            tau_cbf = k_val * (invM' * grad_h.');
        end
    else
        tau_cbf = [0; 0];
    end
    % 总控制输入
    tau_total = tau_nom + tau_cbf;
    % 动力学方程
    qdd = M \ (tau_total - Cqd - g_vec - d_actual);
    dx = [qd; qdd];
end

function h_val = compute_h(q_vec, params)
    % 计算关节角 q_vec 对应的安全函数值 h(q)
    x_end = params.L1 * cos(q_vec(1)) + params.L2 * cos(q_vec(1) + q_vec(2));
    y_end = params.L1 * sin(q_vec(1)) + params.L2 * sin(q_vec(1) + q_vec(2));
    pos = [x_end; y_end];
    h_val = params.r^2 - sum((pos - params.p0).^2);
end
