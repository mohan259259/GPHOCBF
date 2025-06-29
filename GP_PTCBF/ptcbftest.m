% 参数与初始设置
l1 = 1; l2 = 1;                % 连杆长度
m1 = 1; m2 = 1;                % 连杆质量 (假设集中在各杆末端)
r = 0.3;                       % 安全圆半径
p0 = [0.5; 0.5];               % 安全圆中心目标点 p0
q0 = [0; 0]; dq0 = [0; 0];      % 初始关节角和角速度
T_pre = 5;                   % 预设时间 (秒)
c1 = 2; c2 = 3;                % PTCBF 控制系数

% 计算目标关节角 (使末端位于 p0)。取肘下解:
x0 = p0(1); y0 = p0(2);
D = (x0^2 + y0^2 - l1^2 - l2^2) / (2*l1*l2);
q2_target = acos(D);
q1_target = atan2(y0,x0) - atan2(l2*sin(q2_target), l1 + l2*cos(q2_target));
q_target = [q1_target; q2_target];  % 目标关节角

% 定义惯性矩阵 M(q)、科氏力矩阵 C(q,dq)、重力项 G(q)
% (基于简单集中质量模型，无重力)
syms q1 q2 dq1 dq2 real
% 惯性矩阵 (2x2):
M11 = (m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2);
M12 = m2*l2^2 + m2*l1*l2*cos(q2);
M22 = m2*l2^2;
M = [M11, M12; M12, M22];
% 科氏/离心矩阵 (2x2):
h = m2*l1*l2*sin(q2);
C = [ -h*dq2, -h*(dq1+dq2);
       h*dq1,          0      ];
% 将符号函数转为数值匿名函数，方便仿真调用
M_fun = matlabFunction(M, 'Vars', {q1, q2});
C_fun = matlabFunction(C, 'Vars', {q1, q2, dq1, dq2});
G_fun = @(q1,q2) [0; 0];   % 平面无重力

% 定义时间函数 z0(t) 及其一阶导数 dz0(t)
% 此设计确保 z0(t) 在 t -> T_pre 时爆炸，即 z0(t) → +∞
t0_val = 0;   % 初始时刻参数
c_gain = 1;   % 增益参数

z0 = @(t) ( T_pre^2 + c_gain * ( ((t - t0_val).^2) - (t - t0_val)*T_pre ).^2 ) ...
          ./ ( (T_pre + t0_val - t).^2 );
       
dz0 = @(t) ( c_gain * 2 .* ( ((t - t0_val).^2) - (t - t0_val)*T_pre ) ...
         .* (2*(t - t0_val) - T_pre) ...
         .* ((T_pre + t0_val - t).^2) ...
         - ( T_pre^2 + c_gain * ( ((t - t0_val).^2) - (t - t0_val)*T_pre ).^2 ) ...
         .* (-2*(T_pre + t0_val - t)) ) ./ ((T_pre + t0_val - t).^4);
%z0 = @(t) 1./(T_pre - t);
%dz0 = @(t) 1./(T_pre - t).^2;


% 仿真参数
dt = 1e-3;                   % 积分步长
T_total = 10.0;               % 仿真总时长
N = round(T_total/dt);
% 预分配记录数组
time = (0:N)*dt;
q_history = zeros(2, N+1);
h_history = zeros(1, N+1);

% 初始状态
q = q0; dq = dq0;
q_history(:,1) = q0;
% 计算初始安全函数值
p = [l1*cos(q(1)) + l2*cos(q(1)+q(2));
     l1*sin(q(1)) + l2*sin(q(1)+q(2))];
h_history(1) = r^2 - norm(p - p0)^2;

% 仿真主循环（欧拉积分）
for i = 1:N
    t = (i-1)*dt;
    % 计算动力学矩阵
    Mq = M_fun(q(1), q(2));
    Cq = C_fun(q(1), q(2), dq(1), dq(2));
    Gq = G_fun(q(1), q(2));
    
    % 名义 PD 控制器（计算关节空间期望力矩）
    q_err = q - q_target;
    dq_err = dq;   % 目标角速度为 0
    Kp = 5; Kd = 9;   % PD 增益
    u_nom = Mq * (-Kp * q_err - Kd * dq_err) + Cq * dq + Gq;
    
    % 计算 PTCBF 安全约束
    % 计算安全函数及其导数
    p = [l1*cos(q(1)) + l2*cos(q(1)+q(2));
         l1*sin(q(1)) + l2*sin(q(1)+q(2))];
    h_val = r^2 - norm(p - p0)^2;
    
    % 计算 grad_h = ∂h/∂q
    dx_dq1 = -l1*sin(q(1)) - l2*sin(q(1)+q(2));
    dx_dq2 = -l2*sin(q(1)+q(2));
    dy_dq1 =  l1*cos(q(1)) + l2*cos(q(1)+q(2));
    dy_dq2 =  l2*cos(q(1)+q(2));
    dx = p(1) - p0(1); dy = p(2) - p0(2);
    grad_h = -2 * [dx*dx_dq1 + dy*dy_dq1,  dx*dx_dq2 + dy*dy_dq2];
    
    % 计算 Hessian 项对于 dq 的二次型（qd_H_qd = dq^T * H_h * dq）
    d2x_dq1q1 = -l1*cos(q(1)) - l2*cos(q(1)+q(2));
    d2x_dq2q2 = -l2*cos(q(1)+q(2));
    d2x_dq1q2 = -l2*cos(q(1)+q(2));
    d2y_dq1q1 = -l1*sin(q(1)) - l2*sin(q(1)+q(2));
    d2y_dq2q2 = -l2*sin(q(1)+q(2));
    d2y_dq1q2 = -l2*sin(q(1)+q(2));
    H11 = -2 * ((dx_dq1^2 + dy_dq1^2) + dx*d2x_dq1q1 + dy*d2y_dq1q1);
    H22 = -2 * ((dx_dq2^2 + dy_dq2^2) + dx*d2x_dq2q2 + dy*d2y_dq2q2);
    H12 = -2 * ((dx_dq1*dx_dq2 + dy_dq1*dy_dq2) + dx*d2x_dq1q2 + dy*d2y_dq1q2);
    qd = dq;
    qd_H_qd = [qd(1), qd(2)] * [H11, H12; H12, H22] * [qd(1); qd(2)];
    
    % 计算 Barrier 不等式约束的左右项
    % 左项: A * u = grad_h * M^{-1} * u
    A = grad_h / Mq;
    % 右项: b = -[qd_H_qd + grad_h*M^{-1}(d - Cq*dq - Gq) + (c1+c2)*(grad_h*dq + dz0(t)) + c1*c2*(h_val+z0(t))]
    d_val = [0; 0];  % 扰动项，这里设为 0
    b = -( qd_H_qd ...
           + grad_h*(Mq\(d_val - Cq*dq - Gq)) ...
           + (c1+c2)*(grad_h*dq + dz0(t)) ...
           + c1*c2*(h_val + z0(t)) );
       
    % 检查约束是否满足，若不满足则修正控制输入
    if A * u_nom < b
        % 计算调整量 λ
        lambda = (b - A * u_nom) / (A * A');
        u = u_nom + lambda * (A');
    else
        u = u_nom;
    end

    % 状态更新 (Euler 积分)
    ddq = Mq \ (u - Cq*dq - Gq + d_val);
    dq = dq + ddq * dt;
    q  = q + dq * dt;
    
    % 保存数据
    q_history(:, i+1) = q;
    h_history(i+1) = h_val;
end

% 绘制末端执行器轨迹与安全球
px = l1*cos(q_history(1,:)) + l2*cos(q_history(1,:) + q_history(2,:));
py = l1*sin(q_history(1,:)) + l2*sin(q_history(1,:) + q_history(2,:));
figure; hold on; axis equal;
theta = linspace(0, 2*pi, 200);
plot(px, py, 'b-', 'LineWidth', 1.5);
plot(p0(1) + r*cos(theta), p0(2) + r*sin(theta), 'r--', 'LineWidth', 1);
plot(px(1), py(1), 'go', 'MarkerFaceColor','g');        % 起点
plot(p0(1), p0(2), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);  % 目标中心
xlabel('X'); ylabel('Y');
title('末端执行器轨迹及安全区域');
legend('末端轨迹', '安全区域边界', '起点', '目标 p_0');

% 绘制安全函数 h(t) 随时间变化曲线
figure; 
plot(time, h_history, 'b', 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1);
xline(T_pre, 'k:', 'LineWidth', 1);
xlabel('时间 t (s)'); ylabel('h(t)');
title('安全函数 h(t) 随时间的变化');
legend('h(t) = r^2 - ||p - p_0||^2', 'h=0 安全边界', 't = T_{pre}');
