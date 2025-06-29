function dydt = dynamicsAndEst_2DOF(t, y, arm, param, ctrl)
% dynamicsAndEst_2DOF
%   1) 2-DOF机械臂动力学: M(q) qddot + C(q,qd) qd + g(q) = tau + friction + disturbance
%   2) 滤波器 (Pf, bf)
%   3) 信息矩阵 (Q, r)
%   4) 估计律 delta_hat_dot (公式 (35))
%
% state向量: y = [q(2), qd(2), Pf(6), bf(2), Q(1), r(1), delta_hat(1)]
%            其中 Pf是2x3展开(行优先/列优先都行, 需自己定义清晰),
%            Q,r,delta_hat为标量(只估计1维未知参数).
%
% 输出 dydt 同维度.

%% =========== 1. 拆解状态 ===========
q       = y(1:2);
qd      = y(3:4);
Pf      = y(5:10);   % 6元素 => 2x3
bf      = y(11:12);  
Q       = y(13);
r       = y(14);
delta_h = y(15);

% 还原 Pf 为 2x3 矩阵
Pf_mat = reshape(Pf, [2,3]);  % 假设 [pf11 pf12 pf13; pf21 pf22 pf23]

%% =========== 2. 计算机械臂动力学矩阵 M, C, g ===========

% 惯性矩阵 M(q)
M_mat = M_2DOF(q, arm);

% 科氏力矩矩阵 C(q,qd), 使得实际力 = C*qd
C_mat = C_2DOF(q, qd, arm);

% 重力项 g(q)
g_vec = g_2DOF(q, arm);

% 未知摩擦力 = gamma_star * qd
% 但在实现中: gamma_star = gamma_n + delta
%   => friction = (gamma_n + delta) * qd
% 对名义模型 f(x) 我们只放 gamma_n * qd, 剩余 (delta*qd) 视为不确定
delta = param.delta_real;  % (只在对比用, 实际估计器中不会直接调)

% 这里的 disturb 用于添加外部扰动
disturb = param.disturb; 

%% =========== 3. 控制输入 tau ===========
% 简单PD控制, 令期望 q_des=0, qd_des=0
Kp = ctrl.Kp;  Kd = ctrl.Kd;
tau = -Kp*q - Kd*qd;  % PD

%% =========== 4. 系统动力学 求 qddot ===========
%   M(q)*qddot = tau + friction - C(q,qd)*qd - g(q) + disturb
%   friction(真实) = (gamma_n+ delta)*qd
% 令 friction_nominal = gamma_n * qd
%    friction_unknown = delta * qd
% => qddot = M^-1 [ tau + gamma_n*qd + delta*qd - C*qd - g + disturb ]
% => qddot = f(x) + b(x)*tau + M^-1(q)* delta * qd + ...
%
% f(x) = M^-1(q)*( gamma_n*qd - C(q,qd)*qd - g(q) + disturb )
% b(x) = M^-1(q)
%

Mn_inv = inv(M_mat);  % 2x2
f_vec  = Mn_inv * ( param.gamma_n*qd - C_mat*qd - g_vec + disturb );
b_mat  = Mn_inv;

% 真实 qddot (含 delta * qd)
qddot  = f_vec + b_mat*tau + Mn_inv*(delta*qd);

%% =========== 5. 构造滤波器所需项: P_{mu,M}(x) 等 ===========
% 按文献, mu(x,theta_n)= gamma_n * qd,  => partial(mu)/partial(delta)= qd
% => P_{mu}(x) = qd, => P_{mu,M}(x) = M^-1(q)* qd
% (大小: 2x1)
PmuM = Mn_inv * qd;

% 这里要拼到 Pf 里: Pf是2x(2+m)= 2x3 => [ PmuM, I2 ]
%   =>  Pf_mat(:,1) => PmuM(:,1)
%       Pf_mat(:,2:3) => I2 (2x2)
% 但我们用公式(12):  lambda d/dt Pf = -Pf + lambda [PmuM, I2]

%% =========== 6. 构造微分方程: q, qd 的导数 ===========
qdot  = qd;       % trivially
qddot_dot = qddot; % 上面已算好

%% =========== 7. Pf, bf 的微分方程 (12)-(13) ===========
% Pf => 2x3, 令 Pf_mat = [pf_col1, pf_col2, pf_col3]
%    其中 pf_col1 = PmuM, pf_col2= e1, pf_col3= e2 => I2
% => lambda Pf_dot = -Pf + lambda[ PmuM, I2 ]
% => Pf_dot = -(1/lambda)*Pf + [ PmuM, I2 ]
lambda_ = param.lambda;

Pf_mat_dot = -(1/lambda_)*Pf_mat + [PmuM, eye(2)];

% bf => 2x1
% lambda bf_dot = -bf + lambda( f(x)+ b(x)tau ) + qdot
% => bf_dot= -(1/lambda)*bf + [f_vec + b_mat*tau] + (1/lambda)*qdot
bf_dot = -(1/lambda_)*bf + ( f_vec + b_mat*tau ) + (1/lambda_)* qdot;

%% =========== 8. Q, r 的微分方程 (20) ===========
%   Q_dot = -xi*Q + Pf_delta^T Pf_delta
%   r_dot = -xi*r + Pf_delta^T (qdot - bf)
% 其中 Pf_delta = Pf_mat(:,1) 对应前m=1列
xi_ = param.xi;
pf_delta = Pf_mat(:,1);  % 2x1
Q_dot = -xi_ * Q + (pf_delta')*pf_delta; 
r_dot = -xi_ * r + pf_delta'*(qdot - bf); 

%% =========== 9. 估计律 delta_hat_dot (35) ===========
%   delta_hat_dot = -Gamma ( Q delta_hat - r )
Gamma_ = param.Gamma;
delta_hat_dot = -Gamma_ * ( Q*delta_h - r );

%% =========== 10. 整理输出 dydt ===========
% 注意Pf_mat_dot要展成6维
dPf = reshape(Pf_mat_dot,[6,1]);

dydt = [ 
    qdot;         % dq/dt
    qddot_dot;    % dqdot/dt
    dPf;          % dPf/dt => 6x1
    bf_dot;       % dbf/dt => 2x1
    Q_dot;        % dQ/dt
    r_dot;        % dr/dt
    delta_hat_dot % d(delta_hat)/dt
];
end

%% =========== M, C, g 函数示例 ===========

function Mmat = M_2DOF(q, arm)
% 简化2连杆惯性矩阵
q1 = q(1); q2 = q(2);
m1=arm.m1; m2=arm.m2; l1=arm.l1; lc1=arm.lc1; lc2=arm.lc2; I1=arm.I1; I2=arm.I2;

% 常见表达: 
% M = [ I1+I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)), ...
%       I2 + m2*(lc2^2 + l1*lc2*cos(q2))
%     ; ...
%       I2 + m2*(lc2^2 + l1*lc2*cos(q2)), ...
%       I2 + m2*lc2^2 ];
c2 = cos(q2);
M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2);
M12 = I2 + m2*(lc2^2 + l1*lc2*c2);
M21 = M12;
M22 = I2 + m2*lc2^2;
Mmat = [M11, M12; M21, M22];
end

function Cmat = C_2DOF(q, qd, arm)
% 科氏/离心力矩矩阵 
q1 = q(1); q2 = q(2);
qd1=qd(1);qd2=qd(2);
m2=arm.m2; l1=arm.l1; lc2=arm.lc2;

s2 = sin(q2);
h = m2*l1*lc2*s2; 
% 常见表达:
% C = [  0, -h*qd2 ; h*qd1,  0 ]
% 但也可带入 (qd1+qd2)项, 视推导而定
Cmat = [ 0, -h*qd2
         h*qd1, 0 ];
end

function gvec = g_2DOF(q, arm)
% 重力向量
q1 = q(1); q2 = q(2);
m1=arm.m1; m2=arm.m2; g=arm.g; l1=arm.l1; lc1=arm.lc1; lc2=arm.lc2;

g1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1+q2);
g2 = m2*lc2*g*cos(q1+q2);
% 以上若是平面转动, sin->cos 也要看具体连杆定义(你可自行修正)
% 这里仅做示例(竖直平面/转轴?), 有的模型写sin(q), 视坐标系不同
gvec = [g1; g2];
end
