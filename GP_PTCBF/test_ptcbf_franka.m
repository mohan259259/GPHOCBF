function test_ptcbf_franka
% test_ptcbf_franka.m  ----  7‑DoF Franka Panda + Prescribed‑Time CBF
% ------------------------------------------------------------------
% 让 Franka 末端从 (3,3,3) 在 PTCBF 控制下，于 T_s = 2 s 前后安全地进入并
% 保持在半径 r = 1 m、球心 p0 = (0,0,0) 的安全球。
% 需要 Robotics System Toolbox (R2020b+)。
% 与原 3D 质点脚本保持同一符号，只将维度推广到 n = 7。

%% ==================== 全局记录变量 =============================
global tau_nom_history lhs_check_history control_time_history control_counter;

tau_nom_history     = [];
lhs_check_history   = [];
control_time_history = [];
control_counter     = 0;

%% ==================== 机械臂模型 / 维度 ========================
robot  = loadrobot('frankaEmikaPanda','DataFormat','column');  % 使用 column 格式 → 所有关节角都是数值列向量
eeName = 'panda_hand';      % 末端连杆名称
n      = 7;                 % 控制自由度数目 (前7关节)
% 末端连杆，可按 URDF 调整
nArm   = 7;                 % 控制的关节数 (手臂)
% RigidBodyTree 实际包含 9 个非固定关节 (手指各 1)
% 下文统一用 nArm 表示控制维度；机器人相关调用需补足手指角度

%% ==================== 安全球参数 ===============================
r  = 1.0;                    % 安全球半径
p0 = [0;0;0];                % 球心

%% ==================== PTCBF 时间参数 ===========================
T_s     = 2;                % safety deadline
T_theta = 1;                % 论文同式
T_psi   = (T_s + T_theta)/2;
c_gain  = 1.0;
t0_val  = 0.0;              % 参考时刻

%% ==================== CBF & GP 参数 ============================
alpha0 = 1.0; alpha1 = 1.0; beta = 1.0; gamma = 1.0;
mu_fn    = @(x,theta_hat) ones(n,1);   % GP 均值 (示例)
sigma_fn = @(x,theta_hat) 0.5;         % GP 方差 (示例)

%% ==================== 名义 PD 增益 ============================
Kp = 100*eye(n);
Kd =  20*eye(n);

%% ==================== 初始 & 目标关节角 ========================
T_init = trvec2tform([3 3 3]);   % 末端初始位姿 (仅用位置)
T_des  = T_init;                 % 期望保持同一位置

ik      = inverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 1];          % 只关心位置权重
[q0Vec,~]   = ik(eeName,T_init,weights,robot.homeConfiguration);
[qDesVec,~] = ik(eeName,T_des, weights, q0Vec);

q0   = q0Vec(:);                 % 7×1 初始关节角 (数值列向量)
q_des= qDesVec(:);                % 7×1 期望关节角 (数值列向量)
dq0  = zeros(n,1);

x0   = [q0; dq0];                  % 状态向量 14×1

%% ==================== ODE 积分 (分段避奇点) ====================
opts = odeset('RelTol',1e-7,'AbsTol',1e-8);
[t1,x1] = ode45(@robotDyn,[0 1.49], x0,           opts);
[t2,x2] = ode45(@robotDyn,[1.51 1.99], x1(end,:).',opts);
[t3,x3] = ode45(@robotDyn,[2.01 10],   x2(end,:).',opts);

tSol = [t1; t2; t3];
xSol = [x1; x2; x3];

%% ==================== 绘制屏障函数 =============================
hVals = arrayfun(@(k) hFun(xSol(k,1:n).'), 1:length(tSol));

figure('Position',[100 100 1200 480]); hold on; grid on;
plot(tSol,hVals,'LineWidth',2,'DisplayName','h(q(t))');
plot([T_s T_s],ylim,'r--','DisplayName','t = T_s');
xlabel('Time (s)'); ylabel('h(q(t))'); legend('Location','best');

title('Prescribed‑Time CBF on 7‑DoF Franka (Barrier Function)');

%% ==================== 打印结果 ================================
fprintf('\n=========== Simulation done ===========\n');
fprintf('Final t = %.2f s,   h = %.4f\n', tSol(end), hVals(end));
idxTs = find(tSol>=T_s,1);
fprintf('At t = T_s (%.2f s), h = %.4f\n', T_s, hVals(idxTs));
fprintf('======================================\n');

assignin('base','tau_nom_history',tau_nom_history);
assignin('base','lhs_check_history',lhs_check_history);
assignin('base','control_time_history',control_time_history);

%% ==================== 嵌套函数族 ===============================
    function dxdt = robotDyn(t,x)
        q  = x(1:n);
        dq = x(n+1:end);
        tau = controlLaw(t,x);
        ddq = tau;                 % 简化动力学 (M=I)
        dxdt = [dq; ddq];
    end

    function tau = controlLaw(t,x)
        q  = x(1:n);
        dq = x(n+1:end);
        psi_val = psiFun(t,q,dq);
        ah_val  = a_hFun(t,q,dq);
        bh_val  = b_hFun(q);
        mu_val    = mu_fn(x,[]);
        sigma_val = sigma_fn(x,[]);
        amu_val = ah_val + bh_val.'*mu_val - norm(bh_val)*(beta*sigma_val+gamma) + z1Fun(t)*alpha1*psi_val;
        tau_n   = -Kp*(q - q_des) - Kd*dq;
        lhs_chk = amu_val + bh_val.'*tau_n;
        control_counter = control_counter + 1; %#ok<*NASGU>
        control_time_history(control_counter) = t;
        tau_nom_history(:,control_counter)    = tau_n;
        lhs_check_history(control_counter)    = lhs_chk;
        if lhs_chk >= 0
            tau = tau_n;
        else
            denom = bh_val.'*bh_val;
            proj  = eye(n) - (bh_val*bh_val.')/denom;
            tau   = proj*tau_n - (amu_val/denom)*bh_val;
        end
    end

    %% ---------- Barrier & helper functions --------------------
    function val = hFun(q_)
        [p,~] = fk_and_jac(q_);
        val = r^2 - norm(p - p0)^2;
    end

    function dh = dhdqFun(q_)
        [p,J] = fk_and_jac(q_);        % J: 3×7
        dh = -2*J.'*(p - p0);          % 7×1
    end

    function H = d2hdq2Fun(q_)
        eps = 1e-5; H = zeros(n);
        base = dhdqFun(q_);
        for i = 1:n
            dq = zeros(n,1); dq(i)=eps;
            H(:,i) = (dhdqFun(q_+dq) - base)/eps;
        end
    end

    function val = psiFun(t_,q_,dq_)
        val = dhdqFun(q_).'*dq_ + alpha0*z0Fun(t_)*hFun(q_);
    end

    function val = a_hFun(t_,q_,dq_)
        H_ = d2hdq2Fun(q_);
        term1 = dq_.'*H_*dq_;
        term2 = alpha0*z0Fun(t_)*(dhdqFun(q_).'*dq_);
        term3 = alpha0*dz0Fun(t_)*hFun(q_);
        val = term1 + term2 + term3;
    end

    function bh = b_hFun(q_);   bh = dhdqFun(q_);  end

    %% ---------- z0, z1 及其导数 -------------------------------
    function val = z0Fun(t_)
        A = (t_ - t0_val);
        val = (T_s^2 + c_gain*((A^2 - A*T_s)^2)) / (T_s - t_)^2;
    end

    function val = dz0Fun(t_)
        A = (t_ - t0_val); B = (T_s - t_);
        part1 = c_gain*2*(A^2 - A*T_s)*(2*A - T_s)*B^2;
        part2 = (T_s^2 + c_gain*((A^2 - A*T_s)^2))*(-2*B);
        val = (part1 - part2)/(B^4);
    end

    function val = z1Fun(t_)
        A = (t_ - t0_val);
        val = (T_psi^2 + c_gain*((A^2 - A*T_psi)^2)) / (T_psi - t_)^2;
    end

    %% ---------- 正运动学 & Jacobian ---------------------------
    function [p,J] = fk_and_jac(q_)
        T = getTransform(robot,q_,eeName);
        p = T(1:3,4);
        J = geometricJacobian(robot,q_,eeName);
    end
end
