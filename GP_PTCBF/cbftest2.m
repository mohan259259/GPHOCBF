clear; close all; clc;

%% initialization
l1 = 0.333; l2 = 0.316; l3 = 0.384;
d1 = 0.0825; d2 = 0.088; lee = 0.107 + 0.1034;
Slist = Panda_Slist;
Mlist = Panda_Mlist;
Glist = Panda_Glist;
g = [0; 0; -9.81];

% 初始状态（原代码通过 vrep 获取，这里直接赋值）
q = zeros(7, 1);
qdot = zeros(7, 1);
q_start = q; 
qdot_start = qdot;

%% safe region
r = 0.12;

p0 = [0.55; 0.0; 0.2]; 
% 原代码通过 vrep 设置安全区位置，这里直接赋值

%% control parameters
N = 250;

% 不再使用轨迹或PD:
% qDesired = repmat(q_start', N, 1);
% for i = 1: N - 1
%     qdotDesired(i + 1, :)  = wrapToPi(qDesired(i + 1, :) - qDesired(i, :)) / dt;
%     qddotDesired(i + 1, :) = (qdotDesired(i + 1, :) - qdotDesired(i, :)) / dt;
% end
% weights = diag([1, 1, 1, 1, 1, 1, 1]);
% Kp = 30 * weights;
% Kd = 15 * weights;

h1 = zeros(N, 1);
h_bar = zeros(N, 1);
phi1 = zeros(N, 1);
exitflag = ones(N, 1)*0.9;
q_sim = zeros(N, 7);
qdot_sim = zeros(N, 7);
e_sim = zeros(N, 7);
tau_sim = zeros(N, 7);

%% CBF parameters
% 一开始就= true
CBF_switch = true;    
ub = ones(7,1) * 80;
lb = -ub;
alpha1 = 2;
alpha2 = 4;
T_pre = 0.2;       % 规定安全时间
t0_val = 0;        % 仿真开始时刻
c_gain = 2;
phi = @(t) (T_pre^2 + c_gain*((t - t0_val)^2 - (t - t0_val)*T_pre)^2) / ((T_pre + t0_val - t)^2);
phi_dot = @(t) ((c_gain * 2 .* ((t - t0_val).^2 - (t - t0_val)*T_pre) .* (2*(t - t0_val) - T_pre) .* (T_pre + t0_val - t).^2 - (T_pre^2 + c_gain*((t - t0_val).^2 - (t - t0_val)*T_pre).^2) .* (-2*(T_pre + t0_val - t))) ./ (T_pre + t0_val - t).^4);

%% simulation parameters
dt = 4e-3;              % 仿真步长

i = 1; 
i_ptsf = 1;

% 记录末端位置（状态轨迹），用于绘图
p_history = zeros(3, N);

while i <= N
    % 当前仿真时间
    t_current = (i-1)*dt;
    
    % 记录关节状态（单位转换为角度）
    q_sim(i, :) = (q * 180 / pi).';
    qdot_sim(i, :) = (qdot * 180 / pi).';
    
    % 非线性补偿只需要 q, qdot=0 加速度
    M_matrix = MassMatrix(q, Mlist, Glist, Slist);
    nonlinear = InverseDynamics(q, qdot, zeros(7,1), g, zeros(6, 1), Mlist, Glist, Slist);
    
    if CBF_switch
        % 计算末端位置与速度
        [p1, pdot1, A1, B1] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lee);
        p_history(:, i) = p1;  % 记录末端位置
        
        % 不再用 if(h0<=0)
        h = r^2 - norm(p1 - p0)^2;  
        t_current_ptsf = (i_ptsf-1) * dt;
        
        h1(i) = h;
        
        % 添加phi判断：当进入安全区（h1(i)>=0）时，令phi=1, phi_dot=0
        if h1(i) >= 0
            phi_val = 1;
            phi_dot_val = 0;
        else
            phi_val = phi(t_current_ptsf);
            phi_dot_val = phi_dot(t_current_ptsf);
        end
        phi1(i) = phi_val;
        
        C1 = -2 * (p1 - p0).';    % g(x)
        h1_dot = C1 * pdot1;
        h2 = h1_dot + alpha1 * phi(t_current_ptsf) * h1(i);
        D1 = -2*(pdot1.'*pdot1) + alpha1*phi_dot(t_current_ptsf)*h1(i) ...
             + (alpha1+alpha2)*phi(t_current_ptsf)*h1_dot ...
             + alpha1*alpha2*phi(t_current_ptsf)^2*h1(i);
        a = -C1 * A1 / M_matrix;
        b =  C1 * B1 + D1 - (C1*A1 / M_matrix)*nonlinear;
        
        if abs(t_current_ptsf) < 1e-10  % t0 = 0
            calc_expr = -(1/h1(i)) * h1_dot;
            calc_expr_arr(1) = calc_expr;
            disp(['t0 = 0, calc_expr = ', num2str(calc_expr)]);
        end
        
        % quadprog
        H_qp = 2*eye(7);
        f_qp = zeros(7, 1);
        [tau, ~, exitflag(i)] = quadprog(H_qp, f_qp, a, b, [], [], lb, ub);
        i_ptsf = i_ptsf + 1;
    else
        %----原先的 else注释掉
        % tau = M_matrix*(Kp* e + Kd* edot) + nonlinear;
        tau = zeros(7,1);
    end
    
    % 限幅
    tau = Panda_maxtorque(tau);
    tau_sim(i, :) = tau.';
    
    % 计算控制后系统的加速度
    qddot = M_matrix \ (tau - nonlinear);
    
    % Euler 积分更新状态
    qdot = qdot + dt * qddot;
    q = q + dt * qdot;
    
    i = i + 1;
end

%% 绘制 3D 安全区域和状态轨迹
figure;
% 绘制安全球：生成球面数据（50×50 网格），并平移到 p0
[sx, sy, sz] = sphere(50);
surf(r*sx + p0(1), r*sy + p0(2), r*sz + p0(3), 'FaceAlpha', 0.3, 'EdgeColor', 'none');
hold on;
% 绘制状态轨迹（末端位置轨迹）
plot3(p_history(1, :), p_history(2, :), p_history(3, :), 'b', 'LineWidth', 2);
xlabel('x');
ylabel('y');
zlabel('z');
title('3D 状态轨迹与安全区域 (球)');
grid on;
axis equal;
view(3);
legend('安全区域边界', '状态轨迹');

%% 绘制 h(x) 随时间的变化
figure;
plot((0:N-1)*dt, h1, 'LineWidth', 2);
xlabel('时间 (s)');
ylabel('h(x) = r^2 - ||p-p_0||^2');
title('h(x) 随时间变化');
grid on;
hold on;
yline(0, 'r--', 'LineWidth', 1.5);  % 安全边界 h=0
legend('h(x)', '安全边界');

%% plot（原来的力矩图）
figure(1)
plot(1:N, tau_sim(:,1), 'r', 1:N, tau_sim(:,2), 'g', 1:N, tau_sim(:,3), 'b', 1:N, tau_sim(:,4), 'y', ...
     1:N, tau_sim(:,5), 'b', 1:N, tau_sim(:,6), 'c', 1:N, tau_sim(:,7), 'm', ...
     1:N, -40*ones(1, N), '--k', 'linewidth', 1.5);
xlabel('Steps', 'fontsize', 16);
ylabel('$u$', 'interpreter', 'latex', 'fontsize', 18);
legend('u_{1}', 'u_{2}', 'u_{3}', 'u_{4}', 'u_{5}', 'u_{6}', 'u_{7}', 'fontsize', 15);
