function main_estimation_gp_2DOF_oneParam()
% main_estimation_gp_2DOF_oneParam.m
%
% 功能: 在 2自由度机械臂 + 滤波(12,13) + 信息矩阵(20) + 参数估计(35) 的框架下,
%       只估计 1 维未知参数 delta, 并只绘制单条误差曲线 (delta_hat - delta).
%
% 运行:
%   >> main_estimation_gp_2DOF_oneParam
% 观察图中曲线如何随时间收敛.

clc; clear; close all;

%% 1. 机械臂参数(简化示例)
arm.m1 = 1.0;  arm.m2 = 1.0;
arm.l1 = 1.0;  arm.l2 = 1.0;
arm.lc1=0.5;   arm.lc2=0.5;
arm.I1 = 0.05; arm.I2=0.05;
arm.g  = 9.81;

%% 2. 真实 "未建模动力" 与 GP 名义预测
% 假设真实未建模动力: d_real = [ c_actual*sin(q1); 0 ]
% GP 名义预测:         mu_pred = [ c_nom * sin(q1);   0 ]
% => 未知量 delta = c_actual - c_nom (标量)
param.c_actual = 3.0;       % 真正幅值
param.c_nom    = 2.5;       % 名义幅值
param.d_actual = @(q,qd) [ param.c_actual * sin(q(1));
                           0 ];
param.gp_predict_mu = @(q,qd) [ param.c_nom * sin(q(1));
                                0 ];
% 如果需要不确定度, 可定义:
param.gp_predict_sigma = @(q,qd) [0;0];  % 这里先不用

% 真实 delta = c_actual - c_nom
param.delta_true = param.c_actual - param.c_nom;

%% 3. 滤波/估计相关参数
param.dimension_of_delta = 1;  % 只估计1个标量
param.lambda = 5.0;            % 滤波器增益 λ
param.xi     = 0.0;            % 遗忘因子 ξ (设0表示不遗忘)
param.Gamma  = 10.0;           % 估计增益(标量)

%% 4. 控制输入(激励信号)
% 为了让系统充分激励, 用一些正弦/余弦组合
control_input = @(t) [2*sin(0.3*t)+3*cos(1.2*t);
                      1.5*sin(0.5*t)-2*cos(0.8*t)];
param.control_input = control_input;

%% 5. 初始状态
q0    = [pi/6; -pi/8];      % 初始关节角
qd0   = [0; 0];             % 初始关节角速度

% Pf => 大小= 2×(2+dimDelta)= 2×3 => 6个元素
Pf0   = zeros(2, 2 + param.dimension_of_delta);
bf0   = zeros(2,1);
Q0    = zeros(param.dimension_of_delta);   % => [0]
r0    = zeros(param.dimension_of_delta,1); % => [0]
delta_hat0 = 0;   % 初始估计(可以改成其它值)

% 将它们组合成 ODE 的初始状态向量
X0 = [ q0; qd0; Pf0(:); bf0; Q0(:); r0; delta_hat0 ];

%% 6. 数值仿真
tspan = [0 10];
odeopt = odeset('RelTol',1e-6, 'AbsTol',1e-8, 'MaxStep',0.01);
[t_sol, X_sol] = ode45(@(t,x) dynamicsAndEst_gp_2DOF_oneParam(t,x,arm,param),...
                       tspan, X0, odeopt);

% 拆解结果
delta_hat_sol = X_sol(:, end);  % 最后一列是 delta_hat(t)

% 计算单维误差
delta_err = delta_hat_sol - param.delta_true;

%% 7. 绘制单条误差曲线
figure('Name','One-Param Estimation Error','Color','w');
plot(t_sol, delta_err, 'LineWidth',2);
xlabel('Time (s)');
ylabel('Est. Error (\delta_{hat} - \delta)');
title('Unknown Param Estimation Error');
grid on;

disp('=== Simulation Done ===');
disp(['Final param error = ', num2str(delta_err(end))]);
end

%% =====================  ODE函数 ======================
function dX = dynamicsAndEst_gp_2DOF_oneParam(t, X, arm, param)
% dynamicsAndEst_gp_2DOF_oneParam
%   [q(2), qd(2), Pf(2×(2+1)), bf(2), Q(1×1), r(1×1), delta_hat(1)]
%   ODE微分方程

%% 1. 拆解状态
n = 2;                         % 2自由度
dimDelta = param.dimension_of_delta;  % = 1
q  = X(1:2);
qd = X(3:4);

% Pf => 大小=2×(2+dimDelta)=2×3=6个元素
numPf = n*(n+dimDelta);  % 2×3=6
Pf_vec = X(5 : 5+numPf-1);
Pf_mat = reshape(Pf_vec, [n, n+dimDelta]);  % => (2×3)
bf = X(5+numPf : 5+numPf + n -1);

% Q(1×1), r(1×1)
Q_val  = X(5+numPf+n);     % 标量
r_val  = X(5+numPf+n +1);  % 标量

% delta_hat(1×1)
delta_hat = X(end);

%% 2. 机械臂动力学
M_mat = M_2DOF(q, arm);
C_mat = C_2DOF(q, qd, arm);
g_vec = g_2DOF(q, arm);

d_real = param.d_actual(q,qd); % 2x1
tau    = param.control_input(t);

% 真实系统: M*qdd = tau + d_real - C*qd - g
qdd = M_mat \ ( tau + d_real - C_mat*qd - g_vec );

%% 3. GP预测(只做演示)
mu_pred = param.gp_predict_mu(q, qd); % 2x1 (c_nom*sin(q1), 0)
% sigma_pred= param.gp_predict_sigma(q, qd); (若要用可自行加入控制)

%% 4. f(x), b(x) 用于滤波
%   f_vec= [qd; M^-1(mu_pred - C*qd - g)]
f_up   = qd;
f_down = M_mat \ ( mu_pred - C_mat*qd - g_vec );
f_vec  = [f_up; f_down];
b_mat  = [zeros(n); inv(M_mat)];  %#ok<MINV> (4×2)

%% 5. Pf, bf 微分方程 (12), (13)
lambda_ = param.lambda;

% 这里若要真正对 mu_pred 求偏导(相对于 delta), 需写sin(q1)等关系
% 为示例简单 => PmuM= 0(2×1)
phi = [ sin(q(1)); 0 ];
PmuM = inv(M_mat)*phi;  % 2x1


lhs_mat= [PmuM, eye(n)];  % => 2×3
Pf_dot = -(1/lambda_)*Pf_mat + lhs_mat;

bf_down= f_down + (inv(M_mat)*tau);  %#ok<MINV>
bf_dot = -(1/lambda_)*bf + bf_down + (1/lambda_)*qd;

%% 6. 信息矩阵Q, 向量r (20)
xi_   = param.xi;
Pf_del= Pf_mat(:,1:dimDelta); % => 2×1
Q_dot = -xi_*Q_val + Pf_del'*Pf_del;   % 标量
r_dot = -xi_*r_val + Pf_del'*(qd - bf);% 标量

%% 7. 估计律 (35): dot(delta_hat)= -Gamma( Q*delta_hat - r )
Gamma_ = param.Gamma;
delta_hat_dot = -Gamma_*( Q_val*delta_hat - r_val );

%% 8. 汇总导数 dX
dPf = Pf_dot(:);
dX= [
   qd;         % dq
   qdd;        % dqd
   dPf;        % dPf
   bf_dot;     % dbf
   Q_dot;      % dQ
   r_dot;      % dr
   delta_hat_dot  % d(delta_hat)
];
end

%% =========== 附: 2DOF 机械臂动力学函数 ===========
function Mmat = M_2DOF(q, arm)
q1= q(1); q2= q(2);
m1=arm.m1; m2=arm.m2; lc1=arm.lc1; lc2=arm.lc2; I1=arm.I1; I2=arm.I2;
c2= cos(q2);
M11= I1+I2 + m1*lc1^2 + m2*(lc2^2 + 2*lc1*lc2*c2 + lc1^2);
M12= I2 + m2*( lc2^2 + lc1*lc2*c2 );
M21= M12;
M22= I2 + m2*lc2^2;
Mmat= [M11, M12; M21, M22];
end

function Cmat = C_2DOF(q, qd, arm)
q2= q(2); qd1=qd(1); qd2=qd(2);
m2=arm.m2; l1=arm.l1; lc2=arm.lc2; s2= sin(q2);
h= m2*l1*lc2*s2;
Cmat= [0, -h*qd2; h*qd1, 0];
end

function gvec = g_2DOF(q, arm)
q1= q(1); q2= q(2);
m1=arm.m1; m2=arm.m2; g0=arm.g; lc1=arm.lc1; lc2=arm.lc2;
g1= (m1*lc1 + m2*lc1)*g0*cos(q1) + m2*lc2*g0*cos(q1+q2);
g2= m2*lc2*g0*cos(q1+q2);
gvec= [g1; g2];
end
