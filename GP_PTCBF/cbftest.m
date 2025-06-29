clear; close all; clc;

%% 仿真参数
dt = 0.01;              % 时间步长
T = 10;                 % 仿真总时长（秒）
time = 0:dt:T;          % 时间向量

%% 安全集合和CBF参数
% 安全集合: h(x)=1-x1^2-x2^2 >= 0，即单位圆内部
gamma = 2;              % CBF设计参数

%% 名义控制器参数
k_nom = 1;              % 名义控制器增益

%% 初始状态：选择在不安全区域（单位圆外），例如 x = [1.5; 1.5]
x = [1.5; 1.5];

%% 日志记录预分配
x_history = zeros(2, length(time));
h_history = zeros(1, length(time));
u_history = zeros(2, length(time));

%% 仿真循环
for i = 1:length(time)
    % 记录当前状态和 h(x)
    x_history(:, i) = x;
    h_history(i) = 1 - x(1)^2 - x(2)^2;
    
    % 名义控制器：驱动状态向原点收敛
    u_nom = -k_nom * x;
    
    % QP参数构造
    % 目标函数：min ||u - u_nom||^2 => 0.5*u'Hu + f'u, 其中 H = 2I, f = -2u_nom
    H = 2*eye(2);
    f = -2*u_nom;
    
    % CBF约束：2*x' * u <= gamma*(1 - ||x||^2)
    A = 2*x';
    b = gamma*(1 - (x'*x));
    
    % 调用quadprog求解QP (若QP求解失败，退回使用名义控制)
    options = optimoptions('quadprog', 'Display', 'off');
    [u, ~, exitflag] = quadprog(H, f, A, b, [], [], [], [], [], options);
    if exitflag ~= 1
        u = u_nom;
    end
    u_history(:, i) = u;
    
    % 用Euler方法更新状态
    x = x + dt * u;
end

%% 绘制安全函数 h(x) 随时间变化的图像
figure;
plot(time, h_history, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('h(x) = 1 - x_1^2 - x_2^2');
title('CBF: h(x) vs Time');
grid on;
hold on;
yline(0, 'r--', 'LineWidth', 1.2);  % 安全边界 h=0
legend('h(x)', 'Safety Boundary');

%% 绘制状态轨迹图
figure;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 2);  % 绘制单位圆，安全集合边界
hold on;
plot(x_history(1, :), x_history(2, :), 'b', 'LineWidth', 1.5);
xlabel('x_1');
ylabel('x_2');
title('State Trajectory and Safe Set');
axis equal;
grid on;
legend('Safe Set Boundary', 'State Trajectory');
