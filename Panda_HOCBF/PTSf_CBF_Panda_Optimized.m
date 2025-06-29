clear;
clc;
addpath(genpath('..\'));

%% --- 连接 V-REP（CoppeliaSim） ---
disp('Program started');
vrep = remApi('remoteApi'); % 加载远程 API 原型文件
vrep.simxFinish(-1);        % 关闭所有已打开连接
id = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % 连接到 V-REP 服务器

if (id > -1)
    disp('Connected to remote API server');
    
    %% --- 仿真参数设置 ---
    % 为了提高控制频率，将仿真步长 dt 设为 0.01
    dt = 1e-2;  
    vrep.simxSetFloatingParameter(id, vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_blocking);
    vrep.simxSynchronous(id, true);
    vrep.simxStartSimulation(id, vrep.simx_opmode_blocking);
    
    %% --- Panda 机械臂参数初始化 ---
    l1 = 0.333; l2 = 0.316; l3 = 0.384;
    d1 = 0.0825; d2 = 0.088;
    lee = 0.107 + 0.1034;
    omghat_x = [1; 0; 0];
    omghat_y = [0; 1; 0];
    omghat_z = [0; 0; 1];
    Slist = Panda_Slist;   % 假定这些函数已实现
    Mlist = Panda_Mlist;
    Glist = Panda_Glist;
    g_vec = [0; 0; -9.81];
    
    % 初始化工作空间，获取关节句柄
    handles = Panda_workcell_init(vrep, id);
    joint_handles = handles.armJoints;
    q = zeros(7,1);
    qdot = zeros(7,1);
    
    %% --- 障碍物参数 ---
    r = 0.03;               % 障碍物半径
    p0 = [0.53;0;0.2];      % 障碍物位置
    
    %% --- 预计算轨迹（与原代码一致） ---
    action0.qstart = [0;0;0;-90;0;90;45]*pi/180;
    action0.qend   = [0;-40;0;-130;0;90;45]*pi/180;
    q7 = action0.qend(7);
    qa = action0.qend;
    action0.Tf = 1;
    action0.n = action0.Tf/dt + 1;
    action0.method = 5;
    action0.qDesired = JointTrajectory(action0.qstart, action0.qend, action0.Tf, action0.n, action0.method);
    qDesired = action0.qDesired;
    N = action0.n;
    
    Ts = 0.4; ns = Ts/dt;
    qDesired = [qDesired; ones(ns,1)*qDesired(N,:)];
    N = N + ns;
    
    action1.Xstart = Panda_syms_FK(action0.qend, l1, l2, l3, d1, d2, lee);
    [action1.Rstart, action1.pstart] = TransToRp(action1.Xstart);
    action1.Rend = action1.Rstart*MatrixExp3(VecToso3(omghat_z*(0*pi/180)));
    action1.pend = action1.pstart + [0;0;-0.3];
    action1.Xend = RpToTrans(action1.Rend, action1.pend);
    action1.Tf = 1;
    action1.n = action1.Tf/dt + 1;
    action1.method = 5;
    action1.traj = CartesianTrajectory(action1.Xstart, action1.Xend, action1.Tf, action1.n, action1.method);
    action1.qDesired = zeros(7,action1.n);
    for i = 1:action1.n
        action1.qDesired(:,i) = Panda_syms_IK(action1.traj{i}, l1, l2, l3, d1, d2, lee, q7, qa);
        qa = action1.qDesired(:,i);
    end
    qDesired = [qDesired; action1.qDesired'];
    N = N + action1.n;
    
    qDesired = [qDesired; ones(ns,1)*qDesired(N,:)];
    N = N + ns;
    
    action2.Xstart = action1.Xend;
    [action2.Rstart, action2.pstart] = TransToRp(action2.Xstart);
    action2.Rend = action2.Rstart*MatrixExp3(VecToso3(omghat_z*(0*pi/180)));
    action2.pend = action2.pstart + [0.4;0;0];
    action2.Xend = RpToTrans(action2.Rend, action2.pend);
    action2.Tf = 2;
    action2.n = action2.Tf/dt + 1;
    action2.method = 5;
    action2.traj = CartesianTrajectory(action2.Xstart, action2.Xend, action2.Tf, action2.n, action2.method);
    action2.qDesired = zeros(7,action2.n);
    for i = 1:action2.n
        action2.qDesired(:,i) = Panda_syms_IK(action2.traj{i}, l1, l2, l3, d1, d2, lee, q7, qa);
        qa = action2.qDesired(:,i);
    end
    qDesired = [qDesired; action2.qDesired'];
    N = N + action2.n;
    
    qDesired = [qDesired; ones(2*ns,1)*qDesired(N,:)];
    N = N + 2*ns;
    
    %% --- 数据记录变量 ---
    h_bar_record = zeros(N,1);  % 记录 h_bar
    exitflag_record = ones(N,1)*0.9;
    q_sim = zeros(N,7);
    qdot_sim = zeros(N,7);
    e_sim = zeros(N,7);
    tau_sim = zeros(N,7);
    
    %% --- PD 控制器参数 ---
    weights = diag([1,1,1,1,1,1,1]);
    Kp = 25 * weights;  % 稍微调低 PD 增益
    Kd = 10 * weights;
    
    %% --- 预设时间安全（PTSf）相关参数 ---
    % 目标：将时间变化函数 φ(t) 融入 CBF 约束，使得：
    % 2*(p1-p0)'*pdot1 + c*φ(t)*[||p1-p0||^2 - (3*r)^2] >= 0
    CBF_switch = 1;
    c_gain = 8;        % 增大 CBF 增益
    alpha2 = 0.1;      % 调小 alpha2，使得 φ(t) 上升更平缓
    T_pre = 5.0;       % 规定安全时间
    t0_val = 0;        % 仿真开始时刻
    phi = @(t) (T_pre^2 + c_gain*((t - t0_val)^2 - (t - t0_val)*T_pre)^2) / ((T_pre + t0_val - t)^2);
    
    %% --- 预计算轨迹导数 ---
    qdotDesired = zeros(N,7);
    qddotDesired = zeros(N,7);
    for i = 1:N-1
        qdotDesired(i+1,:) = wrapToPi(qDesired(i+1,:) - qDesired(i,:)) / dt;
        qddotDesired(i+1,:) = (qdotDesired(i+1,:) - qdotDesired(i,:)) / dt;
    end
    
    %% --- 控制主循环 ---
    i = 1;
    while vrep.simxGetConnectionId(id) ~= -1
        if i > N
            break;
        end
        
        % 当前仿真时间
        t_current = (i-1)*dt;
        
        % 获取当前关节状态
        for j = 1:7
            [~, q(j)] = vrep.simxGetJointPosition(id, joint_handles(j), vrep.simx_opmode_buffer);
            [~, qdot(j)] = vrep.simxGetObjectFloatParameter(id, joint_handles(j), 2012, vrep.simx_opmode_buffer);
        end
        
        % 计算关节误差
        e = wrapToPi(qDesired(i,:)' - q);
        edot = qdotDesired(i,:)' - qdot;
        
        % 记录数据（角度单位转换）
        q_sim(i,:) = q' * 180/pi;
        qdot_sim(i,:) = qdot' * 180/pi;
        e_sim(i,:) = e' * 180/pi;
        
        % 在每个时刻都计算系统的质量矩阵（保证在各分支均可用）
        M_matrix = MassMatrix(q, Mlist, Glist, Slist);
        % 计算 nominal 控制（即未避障时的力矩）
        tau_nom = M_matrix*(Kp*e + Kd*edot) + ...
                  InverseDynamics(q, qdot, zeros(7,1), g_vec, zeros(6,1), Mlist, Glist, Slist);
        
        if CBF_switch
            % 计算与障碍物相关的变量
            [p1, pdot1, A1, B1] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lee);
            % 预警区设为 (4*r)
            h0 = norm(p1 - p0)^2 - (4*r)^2;
            
            % 计算期望加速度（含 PD 项）
            qddotNorm = qddotDesired(i,:)' + Kp*e + Kd*edot;
            
            % 如果 h0 表示已经进入或接近危险区域，计算 QP 分支控制
            if h0 <= 0
                % QP 控制分支
                H = diag([2,2,2,2,2,2,2]);
                nonlinear = InverseDynamics(q, qdot, qddotDesired(i,:)', g_vec, zeros(6,1), Mlist, Glist, Slist);
                f_qp = -2 * (M_matrix * qddotNorm + nonlinear);
                
                % --- 预设时间安全改动部分 ---
                % 实际障碍屏障函数：用 (1.2*r) 增加冗余
                h_bar = norm(p1 - p0)^2 - (1.2*r)^2;
                h_bar_record(i) = h_bar;
                % 梯度项：取 (p1-p0)' 
                C1 = (p1 - p0)';
                % 速度项：2 * C1 * pdot1
                Lfh_bar = 2 * C1 * pdot1;
                % 计算新的增益 new_alpha = c_gain * φ(t_current)
                new_alpha = c_gain * phi(t_current);
                % 构造 QP 约束： Lfh_bar + new_alpha*h_bar >= 0，并加入模型补偿项
                a_qp = -C1 * A1 / M_matrix;  % 1×7
                b_qp = C1 * B1 + Lfh_bar + new_alpha * h_bar - C1 * (A1/M_matrix * nonlinear);
                b_qp = b_qp(1);  % 转换为标量
                
                lb = -ones(7,1) * 80;
                ub =  ones(7,1) * 80;
                [tau_qp, ~, exitflag] = quadprog(H, f_qp, a_qp, b_qp, [], [], lb, ub);
            else
                % 如果 h0 > 0，说明尚未触发 QP，但为了平滑过渡，我们也计算 QP 控制候选
                % 此处直接采用 nominal 控制作为 QP 分支候选
                tau_qp = tau_nom;
            end
            
            % 引入软切换（线性混合）策略：
            % 定义切换门限（根据实际情况调节，下述阈值仅供参考）
            threshold_danger = -0.001; % h0 <= threshold_danger：完全采用 QP 控制
            threshold_safe   = 0.005;  % h0 >= threshold_safe：完全采用 nominal 控制
            if h0 <= threshold_danger
                tau = tau_qp;
            elseif h0 >= threshold_safe
                tau = tau_nom;
            else
                % 线性混合：权重 w 随 h0 变化
                w = (threshold_safe - h0) / (threshold_safe - threshold_danger);
                tau = w*tau_qp + (1-w)*tau_nom;
            end
        else
            nonlinear = InverseDynamics(q, qdot, qddotDesired(i,:)', g_vec, zeros(6,1), Mlist, Glist, Slist);
            tau = MassMatrix(q, Mlist, Glist, Slist) * (Kp*e + Kd*edot) + nonlinear;
        end
        
        % 对控制输出进行平滑过滤（可选）：例如简单低通滤波
        % 若有 tau_prev，则 tau = 0.8*tau_prev + 0.2*tau;
        % 这里暂不加入，如有需要可自行修改
        
        % 确保 tau 为 7×1 列向量
        tau = reshape(tau, [7,1]);
        tau = Panda_maxtorque(tau);
        tau_sim(i,:) = tau';
        
        % 发送力矩命令到 V-REP 中的机器人
        for j = 1:7
            if tau(j) < 0
                set_vel = -99999;
                set_tau = -tau(j);
            else
                set_vel = 99999;
                set_tau = tau(j);
            end
            vrep.simxSetJointTargetVelocity(id, joint_handles(j), set_vel, vrep.simx_opmode_oneshot);
            vrep.simxSetJointMaxForce(id, joint_handles(j), set_tau, vrep.simx_opmode_oneshot);
        end
        
        vrep.simxSynchronousTrigger(id);
        vrep.simxGetPingTime(id);
        i = i + 1;
    end
    
    %% --- 仿真结束，关闭连接 ---
    vrep.simxStopSimulation(id, vrep.simx_opmode_blocking);
    vrep.simxFinish(id);
else
    disp('Failed connecting to remote API server');
end

vrep.delete(); % 释放资源

%% --- 绘制控制信号 ---
figure(1)
plot(1:N, tau_sim(:,1), 'r', 1:N, tau_sim(:,2), 'g', 1:N, tau_sim(:,3), 'b', ...
     1:N, tau_sim(:,4), 'y', 1:N, tau_sim(:,5), 'b', 1:N, tau_sim(:,6), 'c', ...
     1:N, tau_sim(:,7), 'm', 1:N, -40*ones(1,N), '--k', 'linewidth', 1.5);
xlabel('Steps','fontsize',16);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('u_{1}','u_{2}','u_{3}','u_{4}','u_{5}','u_{6}','u_{7}','fontsize',15);
