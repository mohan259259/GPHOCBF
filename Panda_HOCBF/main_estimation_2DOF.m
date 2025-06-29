function main_estimation_2DOF()
% main_estimation_2DOF
% 演示：基于2自由度机械臂动力学+滤波+公式(35)的未知参数在线估计

clc; clear; close all;

%% ========== 1. 用户可调参数 ==========

% --- 机械臂参数 (示例) ---
arm.m1 = 1.0;       % link1 质量
arm.m2 = 1.0;       % link2 质量
arm.l1 = 0.8;       % link1 长度
arm.l2 = 0.6;       % link2 长度
arm.lc1= 0.4;       % link1 质心到关节距离
arm.lc2= 0.3;       % link2 质心到关节距离
arm.I1 = 0.05;      % link1 转动惯量
arm.I2 = 0.05;      % link2 转动惯量
arm.g  = 9.81;      % 重力加速度

% --- 未知摩擦系数 (param) ---
%   真实值 gamma_star，与名义值 gamma_n 之差为 delta = gamma_star - gamma_n
param.gamma_star = 0.15;     % 真实未知参数
param.gamma_n    = 0.00;     % 名义参数(假设0?)
param.delta_real = param.gamma_star - param.gamma_n;

% --- 滤波/估计相关 ---
param.lambda = 5.0;   % 滤波器参数(lambda)
param.xi     = 0.1;   % 遗忘因子(xi)，若不需遗忘可设0
param.Gamma  = 10.0;  % 估计增益(标量); 若参数多维则是矩阵
% 你可以调大Gamma来加快收敛,但过大会引起振荡

% --- 控制和外部扰动 ---
% 这里简单给个PD控制，使关节跟踪q_des=0
ctrl.Kp = [20, 0; 0, 20];
ctrl.Kd = [ 5, 0; 0,  5];
% 你可根据需要增加更复杂或时变控制
param.disturb = [0;0];  % 常数扰动(若有)

% --- 初始条件 ---
q0    = [0.5; -0.3];        % 初始关节角
qd0   = [0.0;  0.0];        % 初始关节角速度
% Pf是2x3矩阵(因n=2,m=1 => 2x(2+1)=2x3), 用6个数存储
Pf0   = [0; param.lambda; 0; param.lambda; 0; param.lambda]; 
% 这里为演示把pf(0) = [ [0;lambda], [0;lambda], [0;lambda] ]^T
bf0   = [0;0];              % b_f(0)=qdot(0)? 可自己改
Q0    = 0;                  % Q为标量(1x1)
r0    = 0;                  % r也为标量
delta_hat0 = 0;            % 初始估计(可改)

y0 = [q0; qd0; Pf0; bf0; Q0; r0; delta_hat0];

%% ========== 2. 数值积分 ==========

Tend = 10;           % 仿真总时长
odeopt = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t_sol, y_sol] = ode45(@(t,y) dynamicsAndEst_2DOF(t,y,arm,param,ctrl), ...
                       [0 Tend], y0, odeopt);

% 拆解结果
q_sol          = y_sol(:,1:2);
qd_sol         = y_sol(:,3:4);
% Pf_sol(6列)
Pf_sol         = y_sol(:,5:10);
bf_sol         = y_sol(:,11:12);
Q_sol          = y_sol(:,13);
r_sol          = y_sol(:,14);
delta_hat_sol  = y_sol(:,15);

%% ========== 3. 可视化: 估计误差 ==========

figure; 
delta_tilde = delta_hat_sol - param.delta_real; % 估计-真实
plot(t_sol, delta_tilde, 'LineWidth',2);
xlabel('Time (s)'); ylabel('Est. Error (\delta_{hat} - \delta)');
title('Unknown Param Estimation Error');
grid on;

disp('=== Simulation Result ===');
fprintf('  True unknown param (gamma_star) = %.4f\n', param.gamma_star);
fprintf('  Final estimate of param        = %.4f\n', delta_hat_sol(end)+param.gamma_n);
fprintf('  Final error (delta_hat - delta_real) = %.4f\n', delta_tilde(end));
disp('==========================');
end
