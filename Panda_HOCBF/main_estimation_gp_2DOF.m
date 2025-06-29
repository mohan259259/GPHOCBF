function main_estimation_gp_2DOF()
% main_estimation_gp_2DOF
% 演示：2自由度机械臂 + GP 预测(示例) + 自适应参数估计
% 运行后会绘制 (1) 参数估计误差、(2) GP 预测误差

clc; clear; close all;

%% 1. 设置机械臂参数 (简化示例)
arm.m1 = 1.0;  arm.m2 = 1.0;
arm.l1 = 1.0;  arm.l2 = 1.0;
arm.lc1= 0.5;  arm.lc2= 0.5;
arm.I1 = 0.05; arm.I2 = 0.05;
arm.g  = 9.81;

%% 2. 设置(示例)的 "真实" 未建模动态 d(x)
%   这里假设: d1 = c_actual * sin(q1), d2 = offset2(常数)
%   仅用于仿真中产生“真实力矩残差”
param.c_actual   = 8.0;   % 真正的幅度
param.c_nom      = 5.0;   % 名义GP认知 (示例)
param.offset2    = 2.0;   % 关节2的恒定扰动
param.d_actual = @(q,qd) [ param.c_actual*sin(q(1));  param.offset2 ];

%% 3. GP 预测函数: mu(x) + 不确定度 sigma(x)
%   *本示例直接手写一个“假设GP模型”: mu1 = c_nom*sin(q1), mu2=0
%   *sigma1 = 1.0, sigma2=0.5 (示例)
%   在真实应用中, 你可以在这里调用: [mu,sd] = predict(gprMdl,X_new);
param.gp_predict_mu = @(q,qd) [param.c_nom*sin(q(1)); 0];
param.gp_predict_sigma = @(q,qd) [1.0; 0.5];   % 不确定度 (纯演示)

%% 4. 其他滤波/估计相关参数
param.lambda = 5.0;   % 滤波器增益
param.xi     = 0.0;   % 遗忘因子(若需要遗忘可设>0)
param.Gamma  = 10.0;  % 估计增益(标量 or 矩阵)
% 这里假设我们要估计 2 维未知参数: [ c_actual - c_nom; offset2 - 0 ]
% => param.dimension_of_delta = 2;
param.dimension_of_delta = 2;

%% 5. 控制输入 (激励信号)
%   在本示例中直接定义时变力矩, 以激励系统:
control_input = @(t) [ 5*sin(0.5*t)+3*cos(1.2*t);
                       4*sin(0.7*t)-6*cos(0.3*t)];
param.control_input = control_input;

%% 6. 定义初始状态
q0          = [ pi/4; -pi/6 ];
qd0         = [ 0;  0 ];
Pf0         = zeros(2, 2+param.dimension_of_delta);  % Pf是2×(2+m)
bf0         = zeros(2,1);
Q0          = zeros(param.dimension_of_delta);       % m×m
r0          = zeros(param.dimension_of_delta,1);
delta_hat0  = zeros(param.dimension_of_delta,1);     % 初始估计=0

% 把它们打包为列向量
X0 = [q0; qd0; Pf0(:); bf0; Q0(:); r0; delta_hat0];

%% 7. 数值积分
tspan = [0 10];
odeopt = odeset('RelTol',1e-5, 'AbsTol',1e-7, 'MaxStep',0.01);

[t_sol, X_sol] = ode45(@(t,x) dynamicsAndEst_gp_2DOF(t,x,arm,param), tspan, X0, odeopt);

% 拆解状态
%  1-2: q, 3-4: qd, 后续: Pf(2*(2+m)), bf(2), Q(m*m), r(m), delta_hat(m)
q_sol  = X_sol(:,1:2);
qd_sol = X_sol(:,3:4);

% 取最终估计 delta_hat
% 维度= param.dimension_of_delta
dim = param.dimension_of_delta;
delta_hat_sol = X_sol(:, end-dim+1 : end);

%% 8. 计算真实的 delta 和 误差
%   真实delta = [ c_actual - c_nom; offset2 - 0 ]
delta_true = [ param.c_actual - param.c_nom; param.offset2 ];
err_sol = delta_hat_sol - repmat(delta_true', length(t_sol),1);
err_norm = vecnorm(err_sol,2,2);  % ||delta_hat - delta||的二范数

%% 9. GP 预测误差
%   计算 || mu(x) - d(x) ||, 仅用于观察
gp_err = zeros(size(t_sol));
for i = 1:length(t_sol)
    q  = q_sol(i,:)';
    qd = qd_sol(i,:)';
    d_real  = param.d_actual(q, qd);         % [2x1]
    mu_pred = param.gp_predict_mu(q, qd);    % [2x1]
    gp_err(i) = norm(mu_pred - d_real);
end

%% 10. 绘图
figure; 
subplot(2,1,1);
plot(t_sol, err_sol(:,1),'LineWidth',2); hold on;
if dim>1, plot(t_sol, err_sol(:,2),'LineWidth',2); end
xlabel('time(s)'); ylabel('\delta-hat - \delta');
legend('\delta_1 error','\delta_2 error','Location','best');
title('Parameter Estimation Error');
grid on;

subplot(2,1,2);
plot(t_sol, err_norm,'LineWidth',2); hold on;
plot(t_sol, gp_err,'--','LineWidth',2);
xlabel('time(s)'); ylabel('Error norm');
legend('||\delta-hat - \delta||','||\mu - d||','Location','best');
title('Norm of Param Error & GP Prediction Error');
grid on;

disp('========= Simulation Done =========');
final_err = err_sol(end,:);
disp(['Final param error = [', num2str(final_err), ']']);
end
