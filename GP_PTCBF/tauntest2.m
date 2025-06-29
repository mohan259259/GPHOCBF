%% 清除工作区并关闭所有窗口
clear; clc; close all;

%% --- 定义机器人占位函数 ---
% 注意：以下函数仅为占位实现，实际使用时请替换为真实模型

% Panda_M：质量矩阵 7×7（假设为单位阵）
Panda_M = @(q) eye(7);

% Panda_C：科氏/离心项 7×1（假设为零向量）
Panda_C = @(q, qdot) zeros(7,1);

% Panda_g：重力项 7×1（假设为零向量）
Panda_g = @(q) zeros(7,1);

% Panda_FK：正运动学函数，输入 7×1 关节角 q，输出末端位置 p (3×1)
% 为使末端位置较小，这里对计算结果缩放 0.1 倍
Panda_FK = @(q) 0.1 * [ sum(sin(q)); sum(cos(q)); sum(q) ];

% Panda_Jacobian：位置雅可比函数，输入 q，输出 3×7 雅可比矩阵（占位实现）
% 同样将结果缩放 0.1 倍
Panda_Jacobian = @(q) 0.1 * [ cos(q)'; -sin(q)'; ones(1,7) ];

%% --- 参数设置 ---
% 目标末端位置 p₀（3×1），例如：
p0 = [0; 0; 0];

% 安全区半径（检测末端是否处于安全区域），这里设为 1
r_safe = 0.5;

% 末端 PD 控制增益（用于末端位置控制，使末端驱动靠近 p₀）
Kp_e = 20;   % 位置增益
Kd_e = 10;   % 微分增益

%% --- 定义名义控制器 tau_n（匿名函数） ---
% 内部调用局部函数 compute_tau_nominal
tau_n = @(t, x) compute_tau_nominal(x, p0, Kp_e, Kd_e, ...
    Panda_FK, Panda_Jacobian, Panda_M, Panda_C, Panda_g);

%% --- 系统动力学设定 ---
% 状态向量 x = [q; qdot]，q ∈ ℝ⁷，qdot ∈ ℝ⁷，总共 14 维
% 计算力矩法： ddq = inv(Panda_M(q))*(τₙ - Panda_C(q,qdot) - Panda_g(q))
dynamics = @(t, x) [ x(8:14); ...
    Panda_M(x(1:7)) \ ( tau_n(t, x) - Panda_C(x(1:7), x(8:14)) - Panda_g(x(1:7)) ) ];

%% --- 初始状态设定 ---
% 在关节空间中随机生成初始配置 q₀，每个关节均匀采样于 [-π, π]
% 并检测其正运动学转换后的末端位置是否落在以 p₀ 为中心、半径 r_safe 的安全球内
found = false;
while ~found
    q_candidate = -pi + 2*pi*rand(7,1);
   
    p_candidate = Panda_FK(q_candidate);
    if norm(p_candidate - p0) <= r_safe
        found = true;
        q0 = q_candidate;
    end
end
dq0 = zeros(7,1);
x0 = [q0; dq0];

% 显示初始末端位置与 p₀ 的距离
p_init = Panda_FK(q0);
dist0 = norm(p_init - p0);
fprintf('初始末端位置: [%.3f, %.3f, %.3f] 与 p₀ 的距离 = %.3f\n', p_init, dist0);

%% --- 仿真设置 ---
Tsim = 5;          % 仿真时间（秒）
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

%% --- 绘图 ---
figure('Position',[100,100,1200,800]);

% 子图 1：3D 末端轨迹
subplot(2,1,1);
plot3(p_traj(1,:), p_traj(2,:), p_traj(3,:), 'b-', 'LineWidth',2); hold on;
plot3(p0(1), p0(2), p0(3), 'ro', 'MarkerSize',10, 'LineWidth',2); % 目标 p₀
[sx, sy, sz] = sphere(50);
surf(p0(1)+r_safe*sx, p0(2)+r_safe*sy, p0(3)+r_safe*sz, 'FaceAlpha',0.3, 'EdgeColor','none');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('7自由度机械臂末端在名义控制器下趋向 p₀');
axis equal; grid on; view(3);

% 子图 2：末端与 p₀ 距离随时间变化
subplot(2,1,2);
plot(t, distances, 'b-', 'LineWidth',2);
xlabel('Time [s]');
ylabel('Distance |p - p₀|');
title('末端位置与目标 p₀ 之间的距离');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 内部局部函数：计算名义控制器输出 τₙ
function tau = compute_tau_nominal(x, p0, Kp_e, Kd_e, Panda_FK, Panda_Jacobian, Panda_M, Panda_C, Panda_g)
    % 提取关节状态
    q = x(1:7);
    qdot = x(8:14);
    
    % 计算当前末端位置（3×1），利用 Panda_FK
    p = Panda_FK(q);
    
    % 计算末端位置雅可比（3×7），利用 Panda_Jacobian
    Jp = Panda_Jacobian(q);
    
    % 计算当前末端速度
    dp = Jp * qdot;
    
    % 末端位置误差（实际 - 目标）
    err = p - p0;
    
    % 末端 PD 控制律：得到期望末端加速度 a_des
    a_des = -Kp_e * err - Kd_e * dp;
    
    % 利用雅可比伪逆求解期望关节加速度（忽略雅可比导数项）
    ddq_des = pinv(Jp) * a_des;
    
    % 利用计算力矩法得到名义控制器输出：
    % τₙ = M(q)*ddq_des + C(q,qdot) + g(q)
    tau = Panda_M(q) * ddq_des + Panda_C(q, qdot) + Panda_g(q);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
