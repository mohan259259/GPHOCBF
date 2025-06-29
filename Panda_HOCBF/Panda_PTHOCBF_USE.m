clear;
clc;
% addpath(genpath('..\'));

%% connect to vrep
disp('Program started');
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);      % just in case, close all opened connections
id = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % connect to vrep server
% 设置为同步模式
vrep.simxSynchronous(id, true); % 启用同步模式

if (id > -1)
    disp('Connected to remote API server');
    %%  simulaiton parameters setting
    % simulation period
    dt = 4e-2;              % 仿真步长
    vrep.simxSetFloatingParameter(id, vrep.sim_floatparam_simulation_time_step, ...
        dt, vrep.simx_opmode_blocking);
    vrep.simxSynchronous(id, true);
    % start the simulation
    vrep.simxStartSimulation(id, vrep.simx_opmode_blocking);
    
    %%  initialization
    l1 = 0.333;
    l2 = 0.316;
    l3 = 0.384;
    d1 = 0.0825;
    d2 = 0.088;
    lee = 0.107 + 0.1034;
    omghat_x = [1; 0; 0];
    omghat_y = [0; 1; 0];
    omghat_z = [0; 0; 1];
    Slist = Panda_Slist;
    Mlist = Panda_Mlist;
    Glist = Panda_Glist;
    g = [0; 0; -9.81];
    vrep.simxSynchronousTrigger(id);
    handles = Panda_workcell_init(vrep, id);
    joint_handles = handles.armJoints;
    obstacle_handles = handles.obstacle;
    robot_handles = handles.robot;
    tool_handles = handles.tool;
    q = zeros(7, 1);
    qdot = zeros(7, 1);

    %% obstacle
    r = 0.03;
    % p0 = [0.53; 0; 0.2];
    % p0 = [0.57; 0; 0.15];
    [res, pos] = vrep.simxGetObjectPosition(id, obstacle_handles, robot_handles, vrep.simx_opmode_oneshot);
    pause(0.04)
    [res, pos] = vrep.simxGetObjectPosition(id, obstacle_handles, robot_handles, vrep.simx_opmode_buffer);
    % vrchk(vrep, res, true);
    p0 = double(pos.');
    p0 = round(p0, 2);
    %% precomputed trajectory
    action0.qstart = [0; 0; 0; -90; 0; 90; 45] * pi / 180;
    action0.qend = [0; -40; 0; -130; 0; 90; 45] * pi / 180;
    q7 = action0.qend(7);
    qa = action0.qend;
    action0.Tf = 1;
    action0.n = action0.Tf / dt + 1;
    action0.method = 5;
    action0.qDesired = JointTrajectory(action0.qstart, action0.qend, ...
        action0.Tf, action0.n, action0.method);
    qDesired = action0.qDesired;
    N = action0.n;
    % stay still
    Ts = 0.4;
    ns = Ts / dt;
    qDesired = [qDesired; ones(ns, 1) * qDesired(N, :)];
    N = N + ns;
    % action1
    action1.Xstart = Panda_syms_FK(action0.qend, l1, l2, l3, d1, d2, lee);
    [action1.Rstart, action1.pstart] = TransToRp(action1.Xstart);
    action1.Rend = action1.Rstart * MatrixExp3(VecToso3(omghat_z * (0 * pi / 180)));
    action1.pend = action1.pstart + [0; 0; -0.3];
    action1.Xend = RpToTrans(action1.Rend, action1.pend);
    action1.Tf = 1;
    action1.n = action1.Tf / dt + 1;
    action1.method = 5;
    action1.traj = CartesianTrajectory(action1.Xstart, action1.Xend, ...
        action1.Tf, action1.n, action1.method);
    action1.qDesired = zeros(7, action1.n);
    for i = 1: action1.n
        action1.qDesired(:, i) = Panda_syms_IK(action1.traj{i}, ...
            l1, l2, l3, d1, d2, lee, q7, qa);
        qa = action1.qDesired(:, i);
    end
    qDesired = [qDesired; action1.qDesired'];
    N = N + action1.n;
    % stay still
    qDesired = [qDesired; ones(ns, 1) * qDesired(N, :)];
    N = N + ns;
    % action2
    action2.Xstart = action1.Xend;
    [action2.Rstart, action2.pstart] = TransToRp(action2.Xstart);
    action2.Rend = action2.Rstart * MatrixExp3(VecToso3(omghat_z * (0 * pi / 180)));
    action2.pend = action2.pstart + [0.4; 0; 0];
    action2.Xend = RpToTrans(action2.Rend, action2.pend);
    action2.Tf = 2;
    action2.n = action2.Tf / dt + 1;
    action2.method = 5;
    action2.traj = CartesianTrajectory(action2.Xstart, action2.Xend, ...
        action2.Tf, action2.n, action2.method);
    action2.qDesired = zeros(7, action2.n);
    for i = 1: action2.n
        action2.qDesired(:, i) = Panda_syms_IK(action2.traj{i}, ...
            l1, l2, l3, d1, d2, lee, q7, qa);
        qa = action2.qDesired(:, i);
    end
    qDesired = [qDesired; action2.qDesired'];
    N = N + action2.n;
    % stay still longer
    qDesired = [qDesired; ones(2*ns, 1) * qDesired(N, :)];
    N = N + 2*ns;
    
    h1 = zeros(N, 1);
    h_bar = zeros(N, 1);
    phi1 = zeros(N, 1);
    exitflag = ones(N, 1) * 0.9;
    qdotDesired = zeros(N, 7);
    qddotDesired = zeros(N, 7);
    q_sim = zeros(N, 7);
    qdot_sim = zeros(N, 7);
    e_sim = zeros(N, 7);
    tau_sim = zeros(N, 7);

    for i = 1: N - 1
        qdotDesired(i + 1, :) = wrapToPi(qDesired(i + 1, :) - qDesired(i, :)) / dt;
        qddotDesired(i + 1, :) = (qdotDesired(i + 1, :) - qdotDesired(i, :)) / dt;
    end

    %% CBF parameters
    CBF_switch = 1;
    alpha1 = 3;
    alpha2 = 5;
    ub = [1; 1; 1; 1; 1; 1; 1] * 80;
    lb = -ub;
    T_pre = 8.0;       % 规定安全时间
    t0_val = 0;        % 仿真开始时刻
    c_gain = 8;
    phi = @(t) (T_pre^2 + c_gain*((t - t0_val)^2 - (t - t0_val)*T_pre)^2) / ((T_pre + t0_val - t)^2);
    %% PD controller parameters
    weights = diag([1, 1, 1, 1, 1, 1, 1]);
    Kp = 30 * weights;
    Kd = 15 * weights;
    
    %% computed torque control
    i = 1; 
    i_ptsf = 1;
    while vrep.simxGetConnectionId(id) ~= -1
        if i > N
            break
        end
        % 当前仿真时间
        t_current = (i-1)*dt;
        % get states feedback
        for j = 1: 7
            [res, q(j)] = vrep.simxGetJointPosition(id, joint_handles(j), ...
                vrep.simx_opmode_buffer);
            [res, qdot(j)] = vrep.simxGetObjectFloatParameter(id, ...
                joint_handles(j), 2012, vrep.simx_opmode_buffer);
        end
        
        e = wrapToPi(transpose(qDesired(i, :)) - q);
        edot = transpose(qdotDesired(i, :)) - qdot;

        q_sim(i, :) = transpose(q) * 180 / pi;
        qdot_sim(i, :) = transpose(qdot) * 180 / pi;
        e_sim(i, :) = transpose(e) * 180 / pi;

        if CBF_switch
            % 给定关节角度、角速度、urdf，求雅可比矩阵、非线性加速度项和末端的位置与速度
            [~, pdot1, A1, B1] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lee);
            [~, pos1] = vrep.simxGetObjectPosition(id, tool_handles, robot_handles, vrep.simx_opmode_oneshot);
            pause(0.04)
            [~, pos1] = vrep.simxGetObjectPosition(id, tool_handles, robot_handles, vrep.simx_opmode_buffer);
            p1 = double(pos1.');
            p1 = round(p1, 2);
            h0 = norm(p1 - p0)^2 - (5*r)^2;
            p1_recod(:,i) = p1;
            qddotNorm = transpose(qddotDesired(i, :)) + Kp * e + Kd * edot;
            % 提前计算 M_matrix（保证在 if 和 else 均可用）
            M_matrix = MassMatrix(q, Mlist, Glist, Slist);

            if h0 <= 0
                % quadprog
                t_current_ptsf = (i_ptsf-1) * dt;
                % 求目标函数相关参数
                H = diag([2, 2, 2, 2, 2, 2, 2]);
                nonlinear = InverseDynamics(q, qdot, transpose(qddotDesired(i, :)), g, ...
                    zeros(6, 1), Mlist, Glist, Slist);                
                f = -2 * ((M_matrix * qddotNorm + nonlinear));

                % 求约束相关参数
                h1(i) = norm(p1 - p0)^2 - (1.5*r)^2;
                h_bar(i) = norm(p1 - p0)^2 - (1*r)^2;
                C1 = transpose(p1 - p0);
                alpha1_t = alpha1 * phi(t_current_ptsf);
                alpha2_t = alpha2 * phi(t_current_ptsf);
                % alpha1_t = alpha1;
                % alpha2_t = alpha2;
                phi1(i) = 2 * C1 * pdot1 + alpha1_t * h1(i);
                D1 = norm(pdot1)^2 + (alpha1_t + alpha2_t)/2 * phi1(i) ...
                    - alpha1_t^2/2 * h1(i);
                
                a = -C1 * A1 / M_matrix;
                b = C1 * B1 + D1 ...
                    - C1*A1 / M_matrix * nonlinear;

                [tau, fval, exitflag(i)] = quadprog(H, f, a, b, [], [], lb, ub);
                i_ptsf = i_ptsf + 1;
            else
                tau = M_matrix * qddotNorm + InverseDynamics(q, qdot, ...
                    zeros(7, 1), g, zeros(6, 1), Mlist, Glist, Slist);
            end
            
        else
            nonlinear = InverseDynamics(q, qdot, transpose(qddotDesired(i, :)), g, ...
                zeros(6, 1), Mlist, Glist, Slist);
            tau = MassMatrix(q, Mlist, Glist, Slist) ...
                * (Kp * e + Kd * edot) + nonlinear;
        end
        tau = Panda_maxtorque(tau);
        tau_sim(i, :) = transpose(tau);

        % Send torque to vrep
        for j = 1: 7
            if tau(j) < 0
                set_vel = -99999;
                set_tau = -tau(j);
            else
                set_vel = 99999;
                set_tau = tau(j);
            end
            vrep.simxSetJointTargetVelocity(id, joint_handles(j), set_vel, ...
                vrep.simx_opmode_oneshot);
            vrep.simxSetJointMaxForce(id, joint_handles(j), set_tau, ...
                vrep.simx_opmode_oneshot);
        end
        vrep.simxSynchronousTrigger(id);
        vrep.simxGetPingTime(id);
        i = i + 1;
    end
    CBFsuccess = tabulate(exitflag);
    % Now close the connection to vrep
    vrep.simxStopSimulation(id, vrep.simx_opmode_blocking);
    vrep.simxFinish(id);
else
    disp('Failed connecting to remote API server');
end

vrep.delete(); % call the destructor!
figure(1)
plot(1:N,tau_sim(:,1),'r',1:N,tau_sim(:,2),'g',1:N,tau_sim(:,3),'b',1:N,tau_sim(:,4),'y',...
    1:N,tau_sim(:,5),'b',1:N,tau_sim(:,6),'c',1:N,tau_sim(:,7),'m',1:N,-40*ones(1,N),'--k','linewidth',1.5);
xlabel('Steps','fontsize',16);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('u_{1}','u_{2}','u_{3}','u_{4}','u_{5}','u_{6}','u_{7}','fontsize',15);
