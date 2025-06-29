%% 清除工作区并关闭所有窗口
clear; clc; close all;

%% --- 声明全局变量用于记录 h1 数据 ---
global H1_data Time_data;
H1_data = [];    % 用于存储 h1 的历史值
Time_data = [];  % 用于存储对应的时间

%% --- 定义机器人占位函数 ---
% 注意：以下函数仅为占位实现，实际使用时请替换为真实模型

% 质量矩阵 Panda_M：7×7 (这里假设为单位阵)
Panda_M = @(q) eye(7);

% 科氏/离心项 Panda_C：7×1 (假设为零向量)
Panda_C = @(q, qdot) zeros(7,1);

% 重力项 Panda_g：7×1 (假设为零向量)
Panda_g = @(q) zeros(7,1);

% 正运动学 Panda_FK：输入 7×1 关节角 q，输出末端位置 p (3×1)
% 为使末端位置较小，这里对结果缩放 0.1 倍
Panda_FK = @(q) 0.1 * [ sum(sin(q)); sum(cos(q)); sum(q) ];

% 位置雅可比 Panda_Jacobian：输入 q，输出 3×7 雅可比矩阵 (占位实现)
% 这里采用： J_p = 0.1*[cos(q)'; -sin(q)'; ones(1,7)]
Panda_Jacobian = @(q) 0.1 * [ cos(q)'; -sin(q)'; ones(1,7) ];

%% --- 参数设置 ---
p0 = [0; 0; 0];      % 目标末端位置 (3×1)
r_safe = 0.3;        % 安全区半径：末端安全当且仅当 norm(p-p0) < r_safe

Kp_e = 20;           % 末端位置PD控制增益
Kd_e = 10;           % 末端速度PD控制增益

alpha1 = 5;          % CBF 参数 α1
alpha2 = 30;         % CBF 参数 α2

%% --- 定义名义控制器 tau_n（匿名函数） ---
tau_n = @(t, x) compute_tau_nominal(x, p0, Kp_e, Kd_e, ...
    Panda_FK, Panda_Jacobian, Panda_M, Panda_C, Panda_g);

%% --- 系统动力学设定 ---
% 状态 x = [q; qdot]，其中 q ∈ ℝ⁷, qdot ∈ ℝ⁷，总维数 14.
% 简化假设： Panda_M = I, Panda_C = 0, Panda_g = 0 （占位条件）。
% 则 xdot = [qdot; q̈], 且 q̈ = τ，其中 τ 由安全控制函数 safe_control 求解。
dynamics = @(t, x) [ x(8:14); safe_control(t, x) ];

%% --- 初始状态设定 ---
% 在关节空间中随机采样（均匀在 [-π, π]）并加上微小扰动（2°标准差），
% 确保末端位置处于不安全区域 (即：norm(Panda_FK(q)-p0) > r_safe)
found = false;
while ~found
    q_candidate = -pi + 2*pi*rand(7,1);         % 随机采样
    q_candidate = q_candidate + deg2rad(2*randn(7,1)); % 加微小扰动
    p_candidate = Panda_FK(q_candidate);
    if norm(p_candidate - p0) > r_safe   % 不安全区域：距离 > r_safe
        found = true;
        q0 = q_candidate;
    end
end
dq0 = zeros(7,1);
x0 = [q0; dq0];
p_init = Panda_FK(q0);
fprintf('初始末端位置: [%.3f, %.3f, %.3f] 与 p₀ 的距离 = %.3f (应 > r_safe = %.2f)\n', ...
    p_init, norm(p_init - p0), r_safe);

%% --- 仿真设置 ---
Tsim = 6;           % 仿真时间（秒），可延长观察效果
tspan = [0, Tsim];
[t, x] = ode45(dynamics, tspan, x0);

%% --- 计算末端轨迹及误差 ---
nSteps = length(t);
p_traj = zeros(3, nSteps);
for i = 1:nSteps
    q_i = x(i,1:7)';
    p_traj(:, i) = Panda_FK(q_i);
end
distances = vecnorm(p_traj - repmat(p0, 1, nSteps), 2, 1);
% 安全函数 h1 = r_safe^2 - ||p-p0||^2
h1_traj = r_safe^2 - distances.^2;

%% --- 绘图 ---
% 创建 2x2 的布局
tl = tiledlayout(2,2, 'TileSpacing','Compact','Padding','Compact');

% 左侧上方图：末端与 p₀ 距离随时间变化（使用 tile 1）
nexttile(tl, 1);
plot(t, distances, 'b-', 'LineWidth',2);
xlabel('Time [s]');
ylabel('Distance |p - p_0|');
title('距离 p_0');
grid on;

% 左侧下方图：安全函数 h1 随时间变化（使用 tile 3）
nexttile(tl, 3);
plot(t, h1_traj, 'r-', 'LineWidth',2);
xlabel('Time [s]');
ylabel('h_1');
title('安全函数 h_1(t)');
grid on;

% 右侧：3D 末端轨迹，跨越右侧两个格子（tile 2 和 tile 4）
nexttile(tl, [2,1]);  % 让图块跨 2 行 1 列
plot3(p_traj(1,:), p_traj(2,:), p_traj(3,:), 'b-', 'LineWidth',2); 
hold on;
plot3(p0(1), p0(2), p0(3), 'ro', 'MarkerSize',10, 'LineWidth',2);  % 目标 p₀
[sx, sy, sz] = sphere(50);
surf(p0(1)+r_safe*sx, p0(2)+r_safe*sy, p0(3)+r_safe*sz, ...
    'FaceAlpha',0.3, 'EdgeColor','none');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D 末端轨迹');
axis equal;
grid on;
view(3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 内部局部函数：计算名义控制器输出 τₙ
function tau = compute_tau_nominal(x, p0, Kp_e, Kd_e, Panda_FK, Panda_Jacobian, Panda_M, Panda_C, Panda_g)
    q = x(1:7);
    qdot = x(8:14);
    p = Panda_FK(q);
    Jp = Panda_Jacobian(q);
    dp = Jp * qdot;
    err = p - p0;
    a_des = -Kp_e * err - Kd_e * dp;
    ddq_des = pinv(Jp) * a_des;
    tau = Panda_M(q) * ddq_des + Panda_C(q, qdot) + Panda_g(q);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 内部局部函数：安全控制选择函数 safe_control
% 当安全函数 h₁ = r_safe^2 - ||p-p0||^2 < 0 (不安全) 时，利用二阶 CBF，
% 严格采用: h₂ = dot(h₁) + α₁ h₁, h₃ = dot(h₂) + α₂ h₂.
% 用 QP 求解控制输入 τ 使得 h₃ >= 0, 最小化 ||τ||².
function tau = safe_control(t, x)
    % 从主工作区获得必要变量
    persistent alpha1 alpha2 r_safe_local p0_local tau_n_fun Panda_FK_func Panda_Jacobian_func
    if isempty(alpha1)
        alpha1 = evalin('base', 'alpha1');
        alpha2 = evalin('base', 'alpha2');
        r_safe_local = evalin('base', 'r_safe');
        p0_local = evalin('base', 'p0');
        tau_n_fun = evalin('base', 'tau_n');
        Panda_FK_func = evalin('base', 'Panda_FK');
        Panda_Jacobian_func = evalin('base', 'Panda_Jacobian');
    end
    q = x(1:7);
    qdot = x(8:14);
    p = Panda_FK_func(q);
    Jp = Panda_Jacobian_func(q);
    dp = Jp * qdot;
    
    % 计算 Jp_dot：注意 q, qdot 为列向量
    Jp_dot = 0.1 * [ (-sin(q)').* (qdot'); (-cos(q)').* (qdot'); zeros(1,7) ];
    
    % 安全函数 h₁ = r_safe^2 - ||p-p0||^2
    h1 = r_safe_local^2 - norm(p-p0_local)^2;
    % h₁_dot = -2*(p-p0)'*dp
    h1_dot = -2*(p-p0_local)'*dp;
    
    % 定义 h₂ = h₁_dot + alpha1 * h₁
    h2 = h1_dot + alpha1*h1;
    % 注意：这里不显式计算 h₃ 的时间导数，而用整理得到的关于 τ 的约束
    % h₃ >= 0 转化为： -2*(p-p0)'*Jp*τ >= 2*(dp'*dp + (p-p0)'*(Jp_dot*qdot)) - (alpha1+alpha2)*h1_dot - alpha1*alpha2*h1
    % 或 equivalently, 2*(p-p0)'*Jp*τ <= -2*(dp'*dp + (p-p0)'*(Jp_dot*qdot)) + (alpha1+alpha2)*h1_dot + alpha1*alpha2*h1.
    A = 2*(p-p0_local)'*Jp;   % 1×7
    b = -2*(dp'*dp + (p-p0_local)'*(Jp_dot*qdot)) + (alpha1+alpha2)*h1_dot + alpha1*alpha2*h1;
    
    % 添加模式切换的打印，仅修改以下几行即可
    persistent mode
    if isempty(mode)
        mode = -1;
    end
    if h1 >= 0 && mode ~= 1
        fprintf('Switch to tau_n at time %.3f s, h1 = %.3f\n', t, h1);
        mode = 1;
    elseif h1 < 0 && mode ~= 0
        fprintf('Switch to tau_s at time %.3f s, h1 = %.3f\n', t, h1);
        mode = 0;
    end
    
    if h1 >= 0
        % 若安全，则直接采用名义控制器
        tau = tau_n_fun(t, x);
    else
        % QP 求解：最小化 ||tau||^2, subject to A*tau <= b
        H = 2*eye(7);
        f = zeros(7,1);
        options = optimoptions('quadprog','Display','off');
        [tau_qp,~,exitflag] = quadprog(H, f, A, b, [], [], [], [], [], options);
        if exitflag ~= 1
            tau = tau_n_fun(t, x);
        else
            tau = tau_qp;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
