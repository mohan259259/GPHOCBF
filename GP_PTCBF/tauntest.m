%% 清除工作区并关闭所有窗口
clear; clc; close all;

%% 参数设置
% 安全区参数（圆）
p0 = [1; 1];      % 圆心（期望位置）
r_safe = 0.5;     % 安全区半径

% PD 控制器增益
Kp = 10;  % 比例增益
Kd = 5;   % 微分增益

% 动力学设置：这里认为质量矩阵为单位阵，C 和 g 均为零
M = @(q) eye(2);              % 质量矩阵（2×2单位阵）
C = @(q, dq) zeros(2);        % 科氏/离心项（零矩阵）
g_func = @(q) zeros(2, 1);      % 重力项（零向量）

%% 定义名义控制器 tau_n 为匿名函数
% 输入： t（时间）、x = [q; dq] （4×1向量）
% 这里 q = x(1:2), dq = x(3:4)
tau_n = @(t, x) M(x(1:2)) * ( -Kp*(x(1:2) - p0) - Kd*(x(3:4)) ) ...
                 + C(x(1:2), x(3:4)) * (x(3:4)) + g_func(x(1:2));
             
%% 系统闭环动力学
% 系统状态 x = [q; dq]，其中 q 为位置 (2×1), dq 为速度 (2×1)
% 动态方程： xdot = [dq; ddq]，而 ddq = M^{-1} * tau
% 由于 M=I，此处 ddq = tau_n(t,x)
dynamics = @(t, x) [x(3:4); tau_n(t, x)];

%% 设定初始状态
% 随机生成一个位于安全区圆内的起始位置
theta0 = 2*pi*rand;         % 随机角度
rho = r_safe * rand;         % 随机半径（小于 r_safe）
q0 = p0 + [rho*cos(theta0); rho*sin(theta0)];
dq0 = [0; 0];              % 初始速度为 0
x0 = [q0; dq0];

%% 仿真参数
Tsim = 20;                % 仿真时间（秒）
tspan = [0, Tsim];

%% 利用 ODE45 进行仿真
[t, x] = ode45(dynamics, tspan, x0);

%% 合并两个图到一个图中显示
figure('Position',[100, 100, 1200, 800]);

% 子图 1: 末端位置 q 的轨迹
subplot(2,1,1);
plot(x(:,1), x(:,2), 'b-', 'LineWidth', 2); hold on;
plot(p0(1), p0(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % 目标点 p0
% 绘制安全区边界（圆形）
if exist('viscircles', 'file')
    viscircles(p0', r_safe, 'LineStyle', '--');
else
    theta = linspace(0, 2*pi, 100);
    circle_x = p0(1) + r_safe*cos(theta);
    circle_y = p0(2) + r_safe*sin(theta);
    plot(circle_x, circle_y, 'k--', 'LineWidth', 1.5);
end
xlabel('q_1'); ylabel('q_2');
title('末端位置 q 在名义控制器 \tau_n 下趋向 p_0');
axis equal;
grid on;

% 子图 2: 末端位置 q 与目标 p0 的距离随时间变化
subplot(2,1,2);
% 计算每个时刻 q 与 p0 的欧氏距离
distances = vecnorm(x(:,1:2) - repmat(p0', size(x,1), 1), 2, 2);
plot(t, distances, 'b-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Distance |q-p0|');
title('末端位置 q 与目标 p0 的距离随时间变化');
grid on;
