%% 清除工作区并关闭所有窗口
clear; clc; close all;

%% 参数设置（3D 空间）
% 目标点 p0 和安全区参数（安全区域为以 p0 为球心的球体）
p0 = [1; 1; 1];      % 目标位置（3×1向量）
r_safe = 0.5;        % 安全区半径

% PD 控制器增益
Kp = 10;  % 比例增益
Kd = 5;   % 微分增益

% 动力学设置：这里认为质量矩阵为单位阵，C 和 g 均为零
M = @(q) eye(3);                % 质量矩阵（3×3单位阵）
C = @(q, dq) zeros(3);          % 科氏/离心项（零矩阵）
g_func = @(q) zeros(3, 1);        % 重力项（零向量）

%% 定义名义控制器 tau_n 为匿名函数（3D版本）
% 输入： t（时间）、x = [q; dq] （6×1向量，其中 q = x(1:3), dq = x(4:6)）
tau_n = @(t, x) M(x(1:3)) * ( -Kp*(x(1:3) - p0) - Kd*(x(4:6)) ) + ...
                 C(x(1:3), x(4:6)) * (x(4:6)) + g_func(x(1:3));
             
%% 系统闭环动力学
% 状态 x = [q; dq]，其中 q ∈ ℝ³, dq ∈ ℝ³
% 动态方程： xdot = [dq; ddq]，且 ddq = tau_n（因为 M = I）
dynamics = @(t, x) [ x(4:6); tau_n(t, x) ];

%% 设定初始状态（3D）
% 生成一个位于安全区球体内部的随机起始位置
% 利用球坐标生成随机方向和随机半径（取 cube-root 保证体积均匀）
phi = 2*pi*rand;         % 随机方位角
costheta = 2*rand - 1;     % 随机 cos(θ) ∈ [-1, 1]
theta = acos(costheta);    % 由 cos 得 θ
rho = r_safe * rand^(1/3); % 随机半径，保证均匀采样
q0 = p0 + rho * [sin(theta)*cos(phi); sin(theta)*sin(phi); cos(theta)];
dq0 = zeros(3, 1);       % 初始速度为零
x0 = [q0; dq0];

%% 仿真参数
Tsim = 20;                % 仿真时间（秒）
tspan = [0, Tsim];

%% 利用 ODE45 进行仿真
[t, x] = ode45(dynamics, tspan, x0);

%% 合并两个图到一个图中显示（3D轨迹和距离随时间变化的曲线）
figure('Position',[100, 100, 1200, 800]);

% 子图 1: 3D 末端位置 q 的轨迹
subplot(2,1,1);
plot3(x(:,1), x(:,2), x(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(p0(1), p0(2), p0(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % 目标点 p0
% 绘制安全区边界（球体）
% 利用 sphere 函数创建球面数据，显示透明球
[sx, sy, sz] = sphere(50);
surf(p0(1) + r_safe*sx, p0(2) + r_safe*sy, p0(3) + r_safe*sz, ...
    'FaceAlpha', 0.3, 'EdgeColor', 'none');
xlabel('q_1'); ylabel('q_2'); zlabel('q_3');
title('3D 末端位置 q 在名义控制器 \tau_n 下趋向 p_0');
axis equal;
grid on;
view(3);

% 子图 2: 末端位置 q 与目标 p0 的距离随时间变化
subplot(2,1,2);
% 计算每个时刻 q 与 p0 的欧氏距离
distances = vecnorm(x(:,1:3) - repmat(p0', size(x,1), 1), 2, 2);
plot(t, distances, 'b-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Distance |q-p0|');
title('末端位置 q 与目标 p0 的距离随时间变化');
grid on;
