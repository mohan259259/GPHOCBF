clear; close all; clc;

%% 仿真参数
dt = 0.01;              % 时间步长
T = 10;                 % 仿真总时长（秒）
time = 0:dt:T;          % 时间向量

%% 安全集合和CBF参数
R = 1;                  % 安全区域半径
gamma = 2;              % CBF 参数

%% 名义控制器参数
k_nom = 1;              % 名义控制器增益

%% 初始状态：选择在不安全区域（例如 [1.5; 1.5; 1.5] 在球外）
x = [1.5; 1.5; 1.5];

%% 预分配数据记录
x_history = zeros(3, length(time));
h_history = zeros(1, length(time));
u_history = zeros(3, length(time));
nom_used = zeros(1, length(time));  % 记录每一步是否采用了名义控制器（1：采用，0：未采用）

%% 仿真循环
for i = 1:length(time)
    % 记录当前状态
    x_history(:, i) = x;
    
    % 计算安全函数 h(x)=R^2 - ||x||^2
    h = R^2 - (x(1)^2 + x(2)^2 + x(3)^2);
    h_history(i) = h;
    
    % 名义控制器：朝原点收敛
    u_nom = -k_nom * x;
    
    % QP 构造: 最小化 ||u - u_nom||^2
    % 对应形式: 0.5*u'Hu + f'u, 其中 H = 2I, f = -2*u_nom
    H = 2 * eye(3);
    f = -2 * u_nom;
    
    % CBF 约束: -2*x'*u + gamma*h >= 0  => 2*x'*u <= gamma*h
    A = 2 * x';
    b = gamma * h;
    
    % 求解 QP
    options = optimoptions('quadprog', 'Display', 'off');
    [u, ~, exitflag] = quadprog(H, f, A, b, [], [], [], [], [], options);
    if exitflag ~= 1
       u = u_nom;        % 若求解失败，则退回名义控制器
       nom_used(i) = 1;  % 标记使用了名义控制器
       disp(['At time ', num2str(time(i)), ' seconds: Nominal controller used.']);   
    else
       nom_used(i) = 0;
       disp(['At time ', num2str(time(i)), ' seconds: Nominal controller NOT used.']);
    end

    u_history(:, i) = u;
    
    % 添加随机扰动（例如 0.05*randn(3,1)），模拟现实中不规则的轨迹
    noise = 0.05 * randn(3,1);
    
    % Euler 更新状态
    x = x + dt * u + noise * sqrt(dt);  % 用 sqrt(dt) 调整噪声尺度
end

%% 绘制 3D 安全区域和状态轨迹
figure;
% 绘制安全球：生成球面数据（50×50 网格）
[sx, sy, sz] = sphere(50);
surf(R*sx, R*sy, R*sz, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
hold on;
% 绘制状态轨迹
plot3(x_history(1, :), x_history(2, :), x_history(3, :), 'b', 'LineWidth', 2);
xlabel('x_1');
ylabel('x_2');
zlabel('x_3');
title('3D Status Tracks and Safe Zones (Balls)');
grid on;
axis equal;
view(3);
legend('安全区域边界', '状态轨迹');

%% 绘制 h(x) 随时间的变化
figure;
plot(time, h_history, 'LineWidth', 2);
xlabel('时间 (s)');
ylabel('h(x) = R^2 - \|x\|^2');
title('h(x) 随时间变化');
grid on;
hold on;
yline(0, 'r--', 'LineWidth', 1.5);  % 安全边界 h=0
legend('h(x)', '安全边界');

%% 绘制 Nominal Controller 启用情况
figure;
plot(time, nom_used, 'LineWidth', 2);
xlabel('时间 (s)');
ylabel('Nominal Controller Flag');
title('Nominal Controller 是否启用 (1:启用, 0:未启用)');
grid on;
