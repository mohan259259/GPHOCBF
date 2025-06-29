function dydt = ODEfun(t, y, param)
% ODEfun  定义用于 ode45 的微分方程
%
% state = [pf1, pf2, bf, Q, r, delta_hat]
% 这里假设:
%   - x(t) = sin(t), x_dot(t) = cos(t)
%   - f(x)=0, b(x)=1, tau(t)=0 (纯演示用)
%   - P_mu,M(x) = x(t) (对应 partial wrt param), 并附加 I_n => [ x(t), 1 ]

% 拆解状态
pf1       = y(1);
pf2       = y(2);
bf        = y(3);
Q         = y(4);
r         = y(5);
delta_hat = y(6);

% 读取参数
lambda = param.lambda;
xi     = param.xi;
Gamma  = param.Gamma;  % 此处示例将Gamma当标量

% 计算 x(t), x_dot(t)
x_t     = sin(t);
x_dot_t = cos(t);

%% (1) P_f 的微分方程
% 方程 (12) => lambda * Pf_dot = -Pf + lambda*[P_mu,M(x), In]
% => Pf_dot = -(1/lambda)Pf + [P_mu,M(x), In]
% 这里 P_f=(pf1, pf2) 行向量 => pf1 对应 partial w.r.t. param, pf2 对应 identity
pf1_dot = -1/lambda * pf1 + x_t;    % param部分
pf2_dot = -1/lambda * pf2 + 1;      % identity部分

%% (2) b_f 的微分方程
% 方程 (13) => lambda bf_dot = -bf + lambda(f(x)+b(x)*tau) + dot{q}
%           => bf_dot = -(1/lambda)bf + f(x)+b(x)*tau + (1/lambda)*dot{q}
% 这里 f(x)=0, b(x)=1, tau=0 => bf_dot = -(1/lambda)*bf + (1/lambda)*x_dot_t
bf_dot = -1/lambda * bf + (1/lambda)* x_dot_t;

%% (3) Q, r 的微分方程
% 方程 (20):
%   Q_dot = -xi*Q + Pf_delta^T Pf_delta
%   r_dot = -xi*r + Pf_delta^T (dot{q} - b_f)
% 其中 Pf_delta = pf1 (参数维度=1, 取P_f的第1列)
pf_delta = pf1;

Q_dot = -xi * Q + (pf_delta^2);
r_dot = -xi * r + pf_delta*( x_dot_t - bf );

%% (4) 估计律 delta_hat_dot (公式 (35))
%   delta_hat_dot = -Gamma( Q delta_hat - r )
delta_hat_dot = -Gamma * (Q*delta_hat - r);

%% (5) 合并导数
dydt = [
    pf1_dot;     % y(1)
    pf2_dot;     % y(2)
    bf_dot;      % y(3)
    Q_dot;       % y(4)
    r_dot;       % y(5)
    delta_hat_dot % y(6)
];
end
