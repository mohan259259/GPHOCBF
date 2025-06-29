%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% main.m
% 演示一个简化的基于公式 (35) 的在线参数估计仿真
% 可直接运行，观察估计误差是否收敛
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;

%% 1. 设置主要参数

% 真实参数差值 delta_real = theta* - theta_n (仅供对比)
delta_real = -0.2;   % 这里示例设定真实值为 -0.2

% 滤波器和估计增益相关参数
param.lambda = 0.5;  % 滤波器相关系数 lambda
param.xi     = 0.1;  % 遗忘因子 xi (若不想遗忘可设为0)
param.Gamma  = 2.0;  % 估计律增益矩阵 Gamma (简化为标量)

% 其他在 ODE 中可能用到的参数，可根据需求添加
% param.xxx = ...;

%% 2. 设置初始状态
% 状态向量包含 [pf1, pf2, bf, Q, r, delta_hat] 共6个量
% 本示例里，参数维度 m=1 => Q为1x1, r为1x1, delta_hat为1x1
% 令 n=1 => P_f为1x2, b_f为1x1
% 按文献设定 Pf(0) = [0, lambda], bf(0) = 0 or 1 等

pf1_0      = 0;               % P_f第一列初值
pf2_0      = param.lambda;    % P_f第二列初值 (对应文献pf(0)中带lambda的项)
bf_0       = 0;               % b_f初值(可尝试=cos(0)=1等)
Q_0        = 0;               % 信息矩阵 Q 的初值(标量)
r_0        = 0;               % 信息向量 r 的初值(标量)
delta_hat_0= 0;               % 估计的参数差初值

% 将以上打包成初始状态 y0
y0 = [pf1_0; pf2_0; bf_0; Q_0; r_0; delta_hat_0];

%% 3. 调用 ode45 进行仿真
tspan = [0 10];  % 仿真时间，可根据需要延长
[tsol, ysol] = ode45(@(t,y) ODEfun(t,y,param), tspan, y0);

% 解析仿真结果
pf1_sol       = ysol(:,1);
pf2_sol       = ysol(:,2);
bf_sol        = ysol(:,3);
Q_sol         = ysol(:,4);
r_sol         = ysol(:,5);
delta_hat_sol = ysol(:,6);

%% 4. 计算并绘制估计误差
delta_tilde = delta_hat_sol - delta_real;  % 与真实值之差

figure;
plot(tsol, delta_tilde, 'LineWidth', 2);
xlabel('time (s)', 'FontSize',12); 
ylabel('\deltâ - \delta_{real}', 'FontSize',12);
title('Parameter Estimation Error', 'FontSize',14);
grid on;

% 显示最终的估计值与真实值
final_est = delta_hat_sol(end);
disp('========================= RESULTS =========================');
disp(['真实差值 delta_real = ', num2str(delta_real)]);
disp(['最终估计值 delta_hat(T_end) = ', num2str(final_est)]);
disp(['误差 (end) = ', num2str(final_est - delta_real)]);
disp('===========================================================');
