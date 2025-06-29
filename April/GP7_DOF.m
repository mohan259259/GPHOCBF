clear; clc; close all;

%% 定义系统参数
M = eye(7);          % 质量矩阵（7×7）
k = 0.1;             % 科氏力和离心力项的系数（定义 C(x) 的强度）
m_mass = 1.0;        % 每个关节的质量（用于重力项）
g = 9.8;             % 重力加速度
C = @(dq) k * (dq.^2);       % 科氏力和离心力项：逐元素计算 k*dq^2
g_func = @(q) m_mass * g * sin(q);  % 重力项：逐元素计算 m*g*sin(q)
d_true = @(x) sin(x(1:7)) + cos(x(8:14));  % 真实未知动态 d(x)，返回7×1向量
% 仿真参数
tspan = 0:0.01:200;  % 时间范围0到30秒，步长0.01秒

%%% NEW: 修改初始状态（七自由度），前7个为关节位置，后7个为关节速度
x0 = [ones(7,1); ones(7,1)];

%% GP超参数
theta_n = [1.0; 1.0];     % 名义超参数（2维）
theta_true = [0.5; 1.5];  % 真实超参数（2维）
% 计算真实偏差 delta = theta_true - theta_n
delta = theta_true - theta_n;

%% 滤波器参数
lambda = 1.0;     % 滤波时间常数
xi = 1.0;         % 辅助变量衰减率
% 估计器增益
Gamma = [1, 0; 0, 1];   % 估计器增益矩阵

%% 初始化GP模型（输入维数14，输出维数7）
gp = LocalGP_MultiOutput(14, 7, 100, 0.1, 0.5, 1.5);  

%% 初始化训练数据
train_X = [];     % 训练输入（状态 x）
train_y = [];     % 训练输出（未知动态 d(x) 的真实值）

%% 初始化滤波变量
m = length(theta_n);          % 超参数维数 m = 2
% 根据论文 P_f ∈ R^{n×(n+m)}，对于7自由度 n=7，所以 P_f 为 7×(7+2)=7×9
P_f = zeros(7, 7 + m);        
% b_f 对应 R^n，n=7
b_f = zeros(7,1);             

%% 初始化辅助变量
Q = zeros(m, m);  % 辅助变量 Q ∈ R^{2×2}
r = zeros(m, 1);  % 辅助变量 r ∈ R^{2}

%% 初始化超参数估计偏差
delta_hat = zeros(m, 1);  % 估计偏差 δ̂

%% 仿真循环初始化
x = x0;                   % 初始状态（14×1）
t = tspan(1);             % 初始时间
dt = tspan(2) - tspan(1);   % 时间步长

% 存储结果
time_history = [];
state_history = [];
delta_hat_history = [];
d_true_history = [];
mu_history = [];
mu_n_history = [];
sigma_n_history = [];
e_d_history = [];
e_mu_history = []; 
O_history = [];
delta_error_history = [];      % 存储 δ̂ - δ
Q_min_eig_history = [];        % 存储 Q 的最小特征值
P_f_delta_norm_history = [];
V_theta_history = [];          % 李雅普诺夫函数 V_θ 历史
dotV_history = [];             % Vdot 历史

% 用来记录每步 GP 数据量
DataQuantityLog = zeros(size(tspan));

%—————————————————
for i = 1:length(tspan)
    t = tspan(i);

    % 当前状态（分离关节位置和速度）
    q = x(1:7);
    dq = x(8:14);
    current_x = [q; dq];

    % 收集训练数据（每步收集一次）
    if mod(i, 1) == 0
        if gp.DataQuantity >= gp.MaxDataQuantity
            disp(['【Step=', num2str(i), '】删点前DataQuantity=', num2str(gp.DataQuantity)]);
            gp.downdateParam(1);
            disp(['【Step=', num2str(i), '】删点后DataQuantity=', num2str(gp.DataQuantity)]);
        end

        disp(['【Step=', num2str(i), '】加点前DataQuantity=', num2str(gp.DataQuantity)]);
        train_X = [train_X, current_x];
        train_y = [train_y, d_true(current_x)];
        gp.addPoint(current_x, d_true(current_x));
        disp(['【Step=', num2str(i), '】加点后DataQuantity=', num2str(gp.DataQuantity)]);
    end

    % GP预测
    [mu, ~, ~, ~, ~, ~, ~] = gp.predict(current_x);  % 用真实超参数 θ_true 得到 μ(x)
    % 使用 nominal 超参数 theta_n 计算 mu_n
    temp_SigmaF = gp.SigmaF;
    temp_SigmaL = gp.SigmaL;
    gp.SigmaF = theta_n(1);
    gp.SigmaL = theta_n(2);
    [mu_n, sigma_n, ~, ~, ~, ~, ~] = gp.predict(current_x);
    gp.SigmaF = temp_SigmaF;
    gp.SigmaL = temp_SigmaL;
   
    % 计算 P_mu(x) = [dμ/dSigmaF, dμ/dSigmaL]
    if gp.DataQuantity > 0
        delta_theta = 1e-6;
        gp_l_plus = gp; 
        gp_l_plus.SigmaL = gp.SigmaL + delta_theta; 
        [mu_l_plus, ~, ~, ~, ~, ~, ~] = gp_l_plus.predict(current_x);
        dmu_dSigmaL = (mu_l_plus - mu) / delta_theta;
        
        gp_sigma_f_plus = gp;
        gp_sigma_f_plus.SigmaF = gp.SigmaF + delta_theta;
        [mu_sigma_f_plus, ~, ~, ~, ~, ~, ~] = gp_sigma_f_plus.predict(current_x);
        dmu_dSigmaF = (mu_sigma_f_plus - mu) / delta_theta;
        
        P_mu = [dmu_dSigmaF, dmu_dSigmaL];  % 7×2矩阵
    else
        P_mu = zeros(7,2);
    end
    
    P_mu_M = P_mu;

    % 记录建模误差
    d_true_val = d_true(current_x); 
    d_true_history = [d_true_history, d_true_val]; 
    mu_history = [mu_history, mu]; 
    mu_n_history = [mu_n_history, mu_n];
    e_d = d_true_val - mu;  
    e_d_history = [e_d_history, e_d]; 
    e_mu = mu - mu_n;
    e_mu_history = [e_mu_history, e_mu];
    O = e_mu - (P_mu * delta);
    O_history = [O_history, O];
    sigma_n_history = [sigma_n_history, sigma_n];
   
    % 计算 f(x) 和 b(x)
    f_x = M \ (mu - (C(dq) .* dq) - g_func(q)); 
    b_x = 0;                             

    % 更新滤波变量 P_f 和 b_f
    % 修改：ones(7,1) 改为 ones(7,7) 使得 [P_mu_M, ones(7,7)] 尺寸为 7×9，与 P_f 尺寸一致
    P_f_dot = (-P_f + lambda * [P_mu_M, ones(7,7)]) / lambda;
    b_f_dot = (-b_f + lambda * (f_x + b_x) + dq) / lambda;
    P_f = P_f + P_f_dot * dt; 
    b_f = b_f + b_f_dot * dt; 

    % 分解 P_f：前 m 列为 P_f_delta，其余为 P_f_epsilon
    P_f_delta = P_f(:, 1:m);      % 7×2
    P_f_epsilon = P_f(:, m+1:end);  % 7×7
    P_f_delta_norm_history = [P_f_delta_norm_history, norm(P_f_delta, 2)];

    % 更新辅助变量 Q 和 r
    Q_dot = -xi * Q + P_f_delta' * P_f_delta;
    r_dot = -xi * r + P_f_delta' * (dq - b_f);
    Q = Q + Q_dot * dt; 
    r = r + r_dot * dt; 
    
    Q_min_eig = min(eig(Q));
    Q_min_eig_history = [Q_min_eig_history, Q_min_eig];

    % 更新超参数估计
    delta_hat_dot = -Gamma * (Q * delta_hat - r);
    delta_hat = delta_hat + delta_hat_dot * dt; 

    % 计算 δ̂ - δ 和李雅普诺夫函数 V_θ
    delta_error = delta_hat - delta;  
    delta_error_history = [delta_error_history, delta_error];  
    V_theta = 0.5 * sum(delta_error.^2);
    V_theta_history = [V_theta_history, V_theta];
    dotV = -(delta_error' * Q * delta_error) + (delta_error' * r);
    dotV_history = [dotV_history, dotV];

    time_history = [time_history, t];
    state_history = [state_history, x];
    delta_hat_history = [delta_hat_history, delta_hat];

    % 更新系统状态（四阶Runge-Kutta法）
    options = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',dt);
    [t_ode, x_ode] = ode45(@(tt,xx) system_dynamics(tt,xx,d_true,M,C,g_func), [t, t+dt], x, options);
    x = x_ode(end,:)';
    
    DataQuantityLog(i) = gp.DataQuantity;
end

%% 绘制GP数据量随时间变化
figure;
plot(tspan, DataQuantityLog, 'LineWidth',1.2);
xlabel('Time');
ylabel('GP DataQuantity');
title('GP数据量随时间变化');
grid on;

%% 绘制结果
figure('Position',[100,80,1200,800]);
subplot(4,1,1);
plot(time_history, d_true_history, 'b', 'LineWidth', 1.5);
hold on;
plot(time_history, mu_history, 'r--', 'LineWidth', 1.5);
xlabel('time (s)');
ylabel('d(x)');
legend('real d(x)', 'GP prediction \mu(x)');
title('unknown d(x) and GP(theta_true) ');

subplot(4,1,2);
plot(time_history, e_d_history, 'g', 'LineWidth', 1.5);
xlabel('time (s)');
ylabel(' e_d');
title(' e_d = d(x) - \mu(x)');

subplot(4,1,3);
plot(time_history, delta_error_history(1,:), 'b', 'LineWidth', 1.5);
hold on;
plot(time_history, delta_error_history(2,:), 'r', 'LineWidth', 1.5);
xlabel('time (s)');
ylabel('\delta_{\hat{\theta}} - \delta');
legend('\delta_{\hat{\theta},1} - \delta_1', '\delta_{\hat{\theta},2} - \delta_2');
title(' \delta_{\hat{\theta}} - \delta');

subplot(4,1,4);
plot(time_history, Q_min_eig_history, 'm', 'LineWidth', 1.5);
xlabel('time (s)');
ylabel('min Engenv');
title('Q min Engenv');

disp('real theta^*:'), disp(theta_true');
theta_hat = theta_n + delta_hat;
disp('estimated theta_hat:'), disp(theta_hat');
disp('delta:'), disp(theta_true' - theta_hat');

figure('Position',[100, 100, 800, 400]);
subplot(2,1,1);
plot(tspan, V_theta_history, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('V_\theta');
title('Lyapunov Function V_\theta = 0.5||\delta_{error}||^2');
grid on;

subplot(2,1,2);
plot(tspan, dotV_history, 'LineWidth',1.5);
xlabel('Time (s)');
ylabel('V_\theta dot');
title('Lyapunov Function Derivative \dot{V}_\theta');
grid on;



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% system_dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dxdt = system_dynamics(t, x, d_true, M, C, g_func)
    q = x(1:7); 
    dq = x(8:14); 
    %%% NEW: 加入小噪声扰动，防止系统停滞
    ddq = M \ ( - (C(dq) .* dq) - g_func(q) + d_true(x) ) + 0.001*randn(7,1);
    dxdt = [dq; ddq];
end
