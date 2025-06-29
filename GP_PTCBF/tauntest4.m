%% 清除工作区并关闭所有窗口
clear; clc; close all;

%% --- 声明全局变量用于记录 h1 数据 ---
global H1_data Time_data;
H1_data = [];    % 用于存储 h1 的历史值
Time_data = [];  % 用于存储对应的时间

%% --- 定义机器人占位函数 ---
% 注意：以下函数仅为占位实现，实际使用时请替换为真实模型

% 质量矩阵 Panda_M：7×7 (这里假设为单位阵)
Panda_M = @(q) eye(7);

% 科氏/离心项 Panda_C：7×1 (假设为零向量)
Panda_C = @(q, qdot) zeros(7,1);

% 重力项 Panda_g：7×1 (假设为零向量)
Panda_g = @(q) zeros(7,1);

% 正运动学 Panda_FK：输入 7×1 关节角 q，输出末端位置 p (3×1)
% 为使末端位置较小，这里对结果缩放 0.1 倍
Panda_FK = @(q) 0.1 * [ sum(sin(q)); sum(cos(q)); sum(q) ];

% 位置雅可比 Panda_Jacobian：输入 q，输出 3×7 雅可比矩阵 (占位实现)
% 这里采用： J_p = 0.1*[cos(q)'; -sin(q)'; ones(1,7)]
Panda_Jacobian = @(q) 0.1 * [ cos(q)'; -sin(q)'; ones(1,7) ];

%% --- 参数设置 ---
p0 = [0; 0; 0];      % 目标末端位置 (3×1)
r_safe = 0.5;        % 安全区半径：末端安全当且仅当 norm(p-p0) < r_safe

Kp_e = 20;           % 末端位置PD控制增益
Kd_e = 10;           % 末端速度PD控制增益

alpha1 = 5;          % CBF 参数 α1
alpha2 = 10;         % CBF 参数 α2

T_s = 1;  

%% --- 定义名义控制器 tau_n（匿名函数） ---
tau_n = @(t, x) compute_tau_nominal(x, p0, Kp_e, Kd_e, ...
    Panda_FK, Panda_Jacobian, Panda_M, Panda_C, Panda_g);

%% --- 系统动力学设定 ---
% 状态 x = [q; qdot]，其中 q ∈ ℝ⁷, qdot ∈ ℝ⁷，总维数 14.
% 简化假设： Panda_M = I, Panda_C = 0, Panda_g = 0 （占位条件）。
% 则 xdot = [qdot; q̈], 且 q̈ = τ，其中 τ 由安全控制函数 safe_control 求解。
dynamics = @(t, x) [ x(8:14); safe_control(t, x) ];

%% --- 初始状态设定 ---
% 在关节空间中随机采样（均匀在 [-π, π]）并加上微小扰动（2°标准差），
% 确保末端位置处于不安全区域 (即：norm(Panda_FK(q)-p0) > r_safe)
found = false;
while ~found
    q_candidate = -pi + 2*pi*rand(7,1);         % 随机采样
    q_candidate = q_candidate + deg2rad(2*randn(7,1)); % 加微小扰动
    p_candidate = Panda_FK(q_candidate);
    if norm(p_candidate - p0) > r_safe   % 不安全区域：距离 > r_safe
        found = true;
        q0 = q_candidate;
    end
end
dq0 = zeros(7,1);
x0 = [q0; dq0];
p_init = Panda_FK(q0);
fprintf('初始末端位置: [%.3f, %.3f, %.3f] 与 p₀ 的距离 = %.3f (应 > r_safe = %.2f)\n', ...
    p_init, norm(p_init - p0), r_safe);

%% --- 仿真设置 ---
Tsim = 10;           % 仿真时间（秒），可延长观察效果
tspan = [0, Tsim];
[t, x] = ode45(dynamics, tspan, x0);

%% --- 计算末端轨迹及误差 ---
nSteps = length(t);
p_traj = zeros(3, nSteps);
for i = 1:nSteps
    q_i = x(i,1:7)';
    p_traj(:, i) = Panda_FK(q_i);
end
distances = vecnorm(p_traj - repmat(p0, 1, nSteps), 2, 1);
% 安全函数 h1 = r_safe^2 - ||p-p0||^2
h1_traj = r_safe^2 - distances.^2;

%% --- 绘图 ---

fig = figure;
fig.Position = [100, 100, 1200, 800];

% 创建 2x2 的布局
tl = tiledlayout(2,2, 'TileSpacing','Compact','Padding','Compact');

% 左侧上方图：末端与 p₀ 距离随时间变化（使用 tile 1）
nexttile(tl, 1);
plot(t, distances, 'b-', 'LineWidth',2);
xlabel('Time [s]');
ylabel('Distance |p - p_0|');
title('距离 p_0');
grid on;

% 左侧下方图：安全函数 h1 随时间变化（使用 tile 3）
nexttile(tl, 3);
plot(t, h1_traj, 'r-', 'LineWidth',2);
xlabel('Time [s]');
ylabel('h_1');
title('安全函数 h_1(t)');
grid on;

% 右侧：3D 末端轨迹，跨越右侧两个格子（tile 2 和 tile 4）
nexttile(tl, [2,1]);  % 让图块跨 2 行 1 列
plot3(p_traj(1,:), p_traj(2,:), p_traj(3,:), 'b-', 'LineWidth',2); 
hold on;
plot3(p0(1), p0(2), p0(3), 'ro', 'MarkerSize',10, 'LineWidth',2);  % 目标 p₀
[sx, sy, sz] = sphere(50);
surf(p0(1)+r_safe*sx, p0(2)+r_safe*sy, p0(3)+r_safe*sz, ...
    'FaceAlpha',0.3, 'EdgeColor','none');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D 末端轨迹');
axis equal;
grid on;
view(3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 内部局部函数：计算名义控制器输出 τₙ
function tau = compute_tau_nominal(x, p0, Kp_e, Kd_e, Panda_FK, Panda_Jacobian, Panda_M, Panda_C, Panda_g)
    q = x(1:7);
    qdot = x(8:14);
    p = Panda_FK(q);
    Jp = Panda_Jacobian(q);
    dp = Jp * qdot;
    err = p - p0;
    a_des = -Kp_e * err - Kd_e * dp;
    ddq_des = pinv(Jp) * a_des;
    tau = Panda_M(q) * ddq_des + Panda_C(q, qdot) + Panda_g(q);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 内部局部函数：安全控制选择函数 safe_control
% 当安全函数 h₁ = r_safe^2 - ||p-p0||^2 < 0 (不安全) 时，利用二阶 CBF，
% 严格采用: h₂ = dot(h₁) + α₁ h₁, h₃ = dot(h₂) + α₂ h₂.
% 用 QP 求解控制输入 τ 使得 h₃ >= 0, 最小化 ||τ||².
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 内部局部函数：安全控制选择函数 safe_control (带 blow-up z1(t))
function tau = safe_control(t, x)
    % -----------------------------
    % 1) 读入主工作区的必要变量
    persistent alpha1 alpha2 r_safe_local p0_local tau_n_fun ...
               Panda_FK_func Panda_Jacobian_func ...
               T_s eps_z1
    if isempty(alpha1)
        alpha1          = evalin('base','alpha1');
        alpha2          = evalin('base','alpha2');
        r_safe_local    = evalin('base','r_safe');
        p0_local        = evalin('base','p0');
        tau_n_fun       = evalin('base','tau_n');
        Panda_FK_func   = evalin('base','Panda_FK');
        Panda_Jacobian_func = evalin('base','Panda_Jacobian');

        % 假设我们把 "爆炸时间" T_s 和防止 1/0 的小量 eps_z1
        % 也写在主工作区（或者这里直接指定也行）。
        % 下面仅作演示：
        T_s    = 5.0;   % 例如 5秒时进行 blow-up
        eps_z1 = 0.1;   % 小量, 避免分母变0 
    end

    % -----------------------------
    % 2) 计算所需的中间量
    q   = x(1:7);
    qd  = x(8:14);
    p   = Panda_FK_func(q);
    Jp  = Panda_Jacobian_func(q);
    dp  = Jp * qd;

    % 计算 Jp_dot (在你占位实现里即 0.1*[(-sin(q)').*qd'; ...])
    Jp_dot = 0.1 * [ (-sin(q)').*(qd'); 
                     (-cos(q)').*(qd'); 
                      zeros(1,7) ];

    % -----------------------------
    % 3) 定义一级 CBF: h1 & h1_dot
    h1     = r_safe_local^2 - norm(p - p0_local)^2;       % CBF
    h1_dot = -2*(p - p0_local)'*dp;                       % 其导数
    T_s = 2;  
    % -----------------------------
    % 4) 定义 blow-up 函数 z1(t)，这里简单示例：z1(t) = 1 / (T_s - t + eps_z1)
    if t < T_s
        z1 = 1 / (T_s - t + eps_z1);
    else
        z1 = 1e6; % t > T_s 后就固定成很大值; 也可自行设计
    end

    % -----------------------------
    % 5) 改成带 z1(t) 的二阶/三阶 CBF
    %
    % 5.1) 二阶项:  h2 = h1_dot + alpha1 * z1 * h1
    h2 = h1_dot + alpha1 * z1 * h1;

    % 如果你还要做第 3 阶扩展 h3 = h2_dot + alpha2 * z1 * h2，则需要
    % 算出 h2_dot = d/dt(h2)，里面会包含 tau, z1'(t), h1_dot, h1_ddot 等。
    % 这里演示一个“近似做法”，仅替换掉原来的 (alpha1 + alpha2)*h1_dot + alpha1*alpha2*h1
    % 为更复杂的表达式，以适应 z1(t):
    %
    % h2_dot = h1_ddot + alpha1*( z1_dot*h1 + z1*h1_dot )
    % h3     = h2_dot + alpha2*z1*h2
    % ==> h3 >= 0 ==> A*tau <= b
    %
    % 下面仍然是最小展示，直接写出 A 和 b 的形式。真正严谨需你完全展开。
    
    % 先算 h1_ddot（因为它最终也要进约束的右端常数 b）
    % (1) dp_dot = Jp_dot*qd + Jp*qddot = Jp_dot*qd + Jp*tau  (M=I)
    dp_dot = Jp_dot*qd;   % 先不加 Jp*tau, 那会跟 tau 合并成 A*tau
    % h1_ddot = -2 [ dp^T dp + (p-p0)^T dp_dot ]  - 2 (p-p0)^T Jp * tau 
    A_part = -2 * (p - p0_local)' * Jp;  % 关于 tau 的那部分
    B_part = -2*(dp'*dp) - 2*((p - p0_local)'*dp_dot);

    % 现在“近似”地把 h2_dot & h3 的表达式揉进 A 和 b:
    %   h2_dot = (A_part)*tau + B_part + alpha1 [ z1_dot*h1 + z1*(h1_dot) ]
    %   h3 = h2_dot + alpha2*z1*h2
    %       = ...
    %   h3 >= 0  =>  (A_something)*tau <= (b_something)
    %
    % 这里只演示最简单的做法：把原来的
    %   b = -2*(dp'*dp + (p-p0_local)'*(Jp_dot*qdot)) + (alpha1+alpha2)*h1_dot + alpha1*alpha2*h1
    % 全换成包含 z1(t), z1'(t) 等项的更复杂写法。
    
    % z1_dot(t) = d/dt[ 1/(T_s - t + eps_z1) ] = +1 / (T_s - t + eps_z1)^2
    z1_dot = 1 / ( (T_s - t + eps_z1)^2 );
    
    % 令 H1 = alpha1 * [ z1_dot*h1 + z1*h1_dot ]
    %     H2 = alpha2 * z1 * h2
    % h3 = h1_ddot + H1 + H2
    %     = (A_part)*tau + B_part + H1 + H2
    % => A = A_part
    %    b = - [ B_part + H1 + H2 ]
    H1_val = alpha1 * ( z1_dot*h1 + z1*h1_dot );
    H2_val = alpha2 * z1 * h2;  % 注意 h2 里也包含 h1, h1_dot
    
    A = A_part;  % 1×7
    b = - ( B_part + H1_val + H2_val );
    
    % -----------------------------
    % 6) 模式切换与 QP 求解
    persistent mode
    if isempty(mode),  mode = -1; end

    if h1 >= 0
        % 安全区直接用名义控制器
        if mode ~= 1
            fprintf('Switch to tau_n at time %.3f, h1=%.3f\n',t,h1);
            mode = 1;
        end
        tau = tau_n_fun(t, x);
    else
        % 不安全 -> 做 QP 约束
        if mode ~= 0
            fprintf('Switch to tau_s at time %.3f, h1=%.3f\n',t,h1);
            mode = 0;
        end
        H = 2*eye(7);  f = zeros(7,1);
        options = optimoptions('quadprog','Display','off');
        [tau_qp, ~, exitflag] = quadprog(H,f,A,b,[],[],[],[],[],options);
        if exitflag ~= 1
            % 若QP失败, 退回名义控制器
            tau = tau_n_fun(t, x);
        else
            tau = tau_qp;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


