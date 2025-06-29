clear; clc; close all;


% 定义系统参数
M = 1.0;          % 质量
k = 0.1;          % 科氏力和离心力项的系数定义 C(x) 的强度
m = 1.0;          % 质量，用于重力项
g = 9.8;          % 重力加速度
C = @(dq) k * dq^2;  % 科氏力和离心力项，C(x) = k * dq^2，对应论文中的 C(x)
g_func = @(q) m * g * sin(q);  % 重力项g(q) = m * g * sin(q)，对应论文中的 g(q)
d_true = @(x) sin(x(1)) + cos(x(2));  % 真实的未知动态 d(x)
% 仿真参数
tspan = 0:0.01:50;  % 时间范围0到10秒，步长0.01秒，定义仿真时间跨度

%%% NEW: 修改初始状态，让系统更容易动起来
x0 = [1; 1];   

% GP超参数
theta_n = [1.0; 1.0];     % 名义超参数
theta_true = [0.5; 1.5];  % 真实超参数
% 计算真实偏差 delta
delta = theta_true - theta_n;  % delta = theta^* - theta_n

% 滤波器参数
lambda = 1.0;     % 滤波时间常数对应论文中的 λ
xi = 1.0;         % 辅助变量衰减率对应论文中的 ξ
% 估计器增益
Gamma = [1, 0; 0, 1];   % 估计器增益矩阵对角阵 [1, 0; 0, 1]，对应论文中的 Γ
% 初始化GP模型
gp = LocalGP_MultiOutput(2, 1, 100, 0.1, 0.5, 1.5);  
% 初始化训练数据
train_X = [];     % 训练输入存储状态 x
train_y = [];     % 训练输出存储未知动态的真实值
% 初始化滤波变量
m = length(theta_n);  %超参数向量的维数
P_f = zeros(1, 1 + m);  % 滤波变量 P_f= [P_f_delta, P_f_epsilon]，对应论文中的 P_f ∈ R^{n×(n+m)} (公式12)
b_f = 0;               % 滤波变量 b_f对应论文中的 b_f ∈ R^n (公式12)
% 初始化辅助变量
Q = zeros(m, m);  % 辅助变量 Q对应论文中的 Q ∈ R^{m×m} (公式20)
r = zeros(m, 1);  % 辅助变量 r对应论文中的 r ∈ R^m (公式20)
% 初始化超参数估计偏差
delta_hat = zeros(m, 1);  % 估计偏差 delta_hat对应论文中的 \hat{δ} (公式35)
% 仿真循环
x = x0;           % 初始状态
t = tspan(1);     % 初始时间
dt = tspan(2) - tspan(1);  % 时间步长
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
delta_error_history = [];  % 存储 delta_hat - delta
Q_min_eig_history = [];    % 添加：存储 Q 的最小特征值
P_f_delta_norm_history = [];
V_theta_history = [];  % 用于存储李雅普诺夫函数 V_\theta 的历史
dotV_history = [];  % 用于存储每个时刻的 Vdot



% 用来记录每步 GP 数据量
DataQuantityLog = zeros(size(tspan));

%—————————————————

for i = 1:length(tspan)
    t = tspan(i);

    % 当前状态
    q = x(1);
    dq = x(2);
    current_x = [q; dq];

    % 收集训练数据（每1步收集一次）
 
if mod(i, 1) == 0 %当 i 是 1 的倍数时，执行以下代码

    %%% 先检查是否满了，如果满了就打印删点前后DataQuantity
    if gp.DataQuantity >= gp.MaxDataQuantity
        disp(['【Step=', num2str(i), '】删点前DataQuantity=', num2str(gp.DataQuantity)]);
        gp.downdateParam(1);
        disp(['【Step=', num2str(i), '】删点后DataQuantity=', num2str(gp.DataQuantity)]);
    end

    %%% NEW：加点前打印
    disp(['【Step=', num2str(i), '】加点前DataQuantity=', num2str(gp.DataQuantity)]);

    % train_X, train_y 赋值
    train_X = [train_X, current_x];
    train_y = [train_y, d_true(current_x)];

    % 真正执行加点
    gp.addPoint(current_x, d_true(current_x));

    %%% 加点后再打印
    disp(['【Step=', num2str(i), '】加点后DataQuantity=', num2str(gp.DataQuantity)]);

end

    % 计算GP预测
    [mu, ~, ~, ~, ~, ~, ~] = gp.predict(current_x);  % GP均值预测 μ(x, θ_true)
    % 计算 mu_n：使用 nominal 超参数 theta_n 来预测
    temp_SigmaF = gp.SigmaF;
    temp_SigmaL = gp.SigmaL;
    % 切换 gp 超参数为theta_n超参数
    gp.SigmaF = theta_n(1);
    gp.SigmaL = theta_n(2);
    [mu_n, sigma_n, ~, ~, ~, ~, ~] = gp.predict(current_x);  % mu_n = mu(x, theta_n)
    % 恢复原来的超参数
    gp.SigmaF = temp_SigmaF;
    gp.SigmaL = temp_SigmaL;
   







    % 计算 P_mu(x) = [dmu/dl, dmu/dsigma_f]
    if gp.DataQuantity > 0 %判断 GP 是否已有数据
        delta_theta = 1e-6;  % 扰动步长用于数值偏导数
        
        % 对 SigmaL 的偏导数（对应原来的 l）
        gp_l_plus = gp; 
        gp_l_plus.SigmaL = gp.SigmaL + delta_theta; 
        [mu_l_plus, ~, ~, ~, ~, ~, ~] = gp_l_plus.predict(current_x);
        dmu_dSigmaL = (mu_l_plus - mu) / delta_theta; 
        
        % 对 SigmaF 的偏导数（对应原来的 sigma_f）
        gp_sigma_f_plus = gp;
        gp_sigma_f_plus.SigmaF = gp.SigmaF + delta_theta;
        [mu_sigma_f_plus, ~, ~, ~, ~, ~, ~] = gp_sigma_f_plus.predict(current_x);
        dmu_dSigmaF = (mu_sigma_f_plus - mu) / delta_theta;
        
        P_mu = [dmu_dSigmaF, dmu_dSigmaL];
    else
        P_mu = [0, 0];  % GP 无数据时，默认值为零
    end
    
    % 对于单自由度系统，M = 1，因此 P_mu_M = P_mu
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
    f_x = (mu - C(dq) * dq - g_func(q)) / M; 
    b_x = 1 / M;                            

    % 更新滤波变量 P_f 和 b_f
    P_f_dot = (-P_f + lambda * [P_mu_M, 1]) / lambda;
    b_f_dot = (-b_f + lambda * (f_x + b_x * 0) + dq) / lambda;
    P_f = P_f + P_f_dot * dt; 
    b_f = b_f + b_f_dot * dt; 

    % 分解 P_f
    P_f_delta = P_f(1:m); 
    P_f_epsilon = P_f(end); 
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

    % 计算 delta_hat - delta和李雅普诺夫%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    [t_ode, x_ode] = ode45(@(tt,xx) system_dynamics(tt,xx,d_true,M,C,g_func), ...
                           [t, t+dt], x, options);
    x = x_ode(end,:)';
    
    DataQuantityLog(i) = gp.DataQuantity;
end

figure;
plot(tspan, DataQuantityLog, 'LineWidth',1.2);
xlabel('Time');
ylabel('GP DataQuantity');
title('GP数据量随时间变化');
grid on;


% 绘制结果

figure('Position',[100,80,1200,800]);
subplot(4,1,1);
plot(time_history, d_true_history, 'b', 'LineWidth', 1.5);
hold on;
plot(time_history, mu_history, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('d(x)');
legend('Real d(x)', 'GP prediction \mu(x)');
title('unknown dynamics d(x) and GP(theta_true)prediction');

subplot(4,1,2);
plot(time_history, e_d_history, 'g', 'LineWidth', 1.5);
xlabel('time (s)');
ylabel('Modeling error e_d');
title('Modeling error e_d = d(x) - \mu(x)');

subplot(4,1,3);
plot(time_history, delta_error_history(1,:), 'b', 'LineWidth', 1.5);
hold on;
plot(time_history, delta_error_history(2,:), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\delta_{\hat{\theta}} - \delta');
legend('\delta_{\hat{\theta},1} - \delta_1', '\delta_{\hat{\theta},2} - \delta_2');
title('Difference between estimated and true error \delta_{\hat{\theta}} - \delta \delta_{\hat{\theta}} - \delta');

subplot(4,1,4);
plot(time_history, Q_min_eig_history, 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Min λQ');
title('The trajectory of the smallest eigenvalue of Q');

disp('真实超参数 theta^*:'), disp(theta_true');
theta_hat = theta_n + delta_hat;
disp('估计超参数 theta_hat:'), disp(theta_hat');
disp('误差delta:'), disp(theta_true' - theta_hat');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Position',[100, 100, 800, 400]);  % 调整窗口尺寸，使得垂直空间足够
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




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%上界的计算和验证%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 计算 nominal delta（theta_true - theta_n） 的范数（上界）
%delta_error_norm = vecnorm(delta_error_history, 2, 1);  % 得到一个 1×N 的向量，每个元素为该时刻的范数
%delta_error_upper_bound = max(delta_error_norm);
%fprintf('delta error 范数的上界为： %f\n', delta_error_upper_bound);

% 计算每个时间步 O 的欧氏范数
O_norm = vecnorm(O_history, 2, 1);  % 得到一个 1×N 的向量
% 计算 ||O|| 的上界（nom），即所有时间步中的最大值
O_norm_upper_bound = max(O_norm);
fprintf('||O|| 的上界（nom）为： %f\n', O_norm_upper_bound);

sigma_n_upper_bound = max(sigma_n_history);
fprintf('sigma_n 的上界为： %f\n', sigma_n_upper_bound);

L_mu_M = max(P_f_delta_norm_history);
fprintf('L_{mu,M} = %f\n', L_mu_M);

fprintf('Q 的最小特征值下界(忽略第一个值)为： %f\n', Q_min_eig);


%%%%%%%%%%%%%%%%估计 L_{sigma,theta} 的示例代码 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1) 设置采样范围（根据你需要的 \theta 空间）
theta1_vals = linspace(0.1, 1.0, 10);  % 对应 SigmaF 的取值范围
theta2_vals = linspace(0.5, 2.0, 10);   % 对应 SigmaL 的取值范围

% 2) 选定若干 x 用于测试（可多点）
x_candidates = [0; 0];   % 此处选取 x = [0; 0]
% 若需要多个测试点可使用如下格式：
% x_candidates = [[0;0], [1;2], [2;3]];

h = 1e-6;            % 有限差分步长
L_sigma_theta = 0;   % 初始化最大梯度范数

% 备份原来的超参数，防止修改后影响主流程
original_SigmaF = gp.SigmaF;
original_SigmaL = gp.SigmaL;

for xi = 1:size(x_candidates,2)
    x_test = x_candidates(:, xi);
    for a = theta1_vals
        for b = theta2_vals
            % 设置 gp 当前超参数 = (a, b)
            gp.SigmaF = a;
            gp.SigmaL = b;
            
            % 调用 predict 得到预测方差 sigma_val
            [~, sigma_val, ~, ~, ~, ~, ~] = gp.predict(x_test);
            
            % 对每个超参数维度做有限差分计算梯度
            grad_theta = zeros(2,1);  % 因为 theta 是二维向量 [SigmaF; SigmaL]
            for iParam = 1:2
                theta_pert = [a; b];
                theta_pert(iParam) = theta_pert(iParam) + h;
                
                % 设置扰动后的超参数
                gp.SigmaF = theta_pert(1);
                gp.SigmaL = theta_pert(2);
                
                % 计算扰动后预测方差 sigma_val_pert
                [~, sigma_val_pert, ~, ~, ~, ~, ~] = gp.predict(x_test);
                
                % 用有限差分近似梯度分量
                grad_theta(iParam) = (sigma_val_pert - sigma_val) / h;
            end
            
            % 计算该采样点的梯度范数
            grad_norm = norm(grad_theta);
            
            % 更新最大梯度范数
            if grad_norm > L_sigma_theta
                L_sigma_theta = grad_norm;
            end
        end
    end
end

% 恢复原来的超参数
gp.SigmaF = original_SigmaF;
gp.SigmaL = original_SigmaL;

fprintf('估计得到的 L_{sigma,theta} = %f\n', L_sigma_theta);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%eps_bar_partial
beta = 1;
delta_bar = 2;  
eps_bar_partial = O_norm_upper_bound + beta * sigma_n_upper_bound + beta * L_sigma_theta * delta_bar;
fprintf('eps_bar_partial = %f\n', eps_bar_partial);

%eps_bar
eps_bar = (1 / xi) * L_mu_M * eps_bar_partial; 
fprintf('eps_bar = %f\n', eps_bar);

%omega_bar
omega_bar = 0.5 / (Q_min_eig^2);
fprintf('\\omega_bar = %f\n', omega_bar);

%chi
chi = Q_min_eig;
fprintf('\\chi = %f\n', chi);

%eps_bar_theta
term1 = abs((2 * delta_bar)^2/(2 * omega_bar * eps_bar^2) - 1);
term2 = abs((2 * delta_bar)^2/(2 * 1) - omega_bar * eps_bar^2);
term3 = (1 / term1) * term2 + omega_bar * eps_bar^2;
eps_bar_theta = sqrt(term3 / omega_bar);
fprintf('eps_bar_theta = %f\n', eps_bar_theta);

delta_tilde_norm_upper_bound = (1 / Q_min_eig) * eps_bar_theta;
fprintf('||delta_tilde(t)||_upper_bound = %f\n', delta_tilde_norm_upper_bound);

delta_tilde_norm = norm(delta_error);
fprintf('||delta_tilde(t)|| = %f\n', delta_tilde_norm);

if delta_tilde_norm < delta_tilde_norm_upper_bound
    fprintf('真实的delta_tilde_norm符合要求，小于上界，在界内。\n');
else
    fprintf('真实的delta_tilde_norm不符合，不在界内。\n');
end

dotV_upper_bound = - Q_min_eig * V_theta + (1 / Q_min_eig) * (eps_bar^2 / 2);
fprintf('dotV_upper_bound = %f\n', dotV_upper_bound);
fprintf('dotV = %f\n', dotV);

if dotV < dotV_upper_bound
    fprintf('真实的dotV符合要求，小于上界，在界内。\n');
else
    fprintf('真实的dotV不符合，不在界内。\n');
end

V_theta_upper_bound = term3
fprintf('V_theta_upper_bound = %f\n', V_theta_upper_bound);
fprintf('V_theta = %f\n', V_theta);

if V_theta < V_theta_upper_bound
    fprintf('真实的V_theta符合要求，小于上界，在界内。\n');
else
    fprintf('真实的V_theta不符合，不在界内。\n');
end




% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% system_dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dxdt = system_dynamics(t, x, d_true, M, C, g_func)
    q = x(1); 
    dq = x(2); 

    %%% NEW: 人为加点小噪声扰动，防止系统“停”在平衡
    ddq = ( - C(dq) * dq - g_func(q) + d_true([q; dq]) ) / M + 0.001*randn(1);

    dxdt = [dq; ddq];
end





