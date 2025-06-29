clear; close all; clc;

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
    l1 = 0.333; l2 = 0.316; l3 = 0.384;
    d1 = 0.0825; d2 = 0.088; lee = 0.107 + 0.1034;
    Slist = Panda_Slist;
    Mlist = Panda_Mlist;
    Glist = Panda_Glist;
    g = [0; 0; -9.81];
    vrep.simxSynchronousTrigger(id);
    handles = Panda_workcell_init(vrep, id);
    joint_handles = handles.armJoints;
    obstacle_handles = handles.obstacle;
    tool_handles = handles.tool;
    safe_region_handles = handles.safe_region;    
    
     %% Get Start State
    q = zeros(7, 1);
    qdot = zeros(7, 1);
    for j = 1:7
        [~, q(j)] = vrep.simxGetJointPosition(id, joint_handles(j), vrep.simx_opmode_buffer);
        [~, qdot(j)] = vrep.simxGetObjectFloatParameter(id, joint_handles(j), 2012, vrep.simx_opmode_buffer);
    end
    q_start = q; 
    qdot_start = qdot;

    %% safe region
    r = 0.12;
    [~, pos] = vrep.simxGetObjectPosition(id, safe_region_handles, -1, vrep.simx_opmode_blocking);
    p0 = double(pos.');

    %% control parameters
    N = 50;
    qDesired = repmat(q_start', N, 1);     
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

    % weights = diag([1, 1, 1, 1, 1, 1, 1]);
    % Kp = 0 * weights;
    % Kd = 0 * weights;

    %% CBF parameters
    CBF_switch = true;    
    ub = ones(7,1) * 80;
    lb = -ub;
    alpha1 = 2;
    alpha2 = 4;
    T_pre = 8.0;       % 规定安全时间
    t0_val = 0;        % 仿真开始时刻
    c_gain = 8;
    phi = @(t) (T_pre^2 + c_gain*((t - t0_val)^2 - (t - t0_val)*T_pre)^2) / ((T_pre + t0_val - t)^2);
    phi_dot = @(t) (c_gain*(4*(t-t0_val)^3+2*T_pre^2*(t0_val)^2-6*(t-t0_val)^2*T_pre)*(T_pre+t0_val-t)^(-2) ...
                    + (T_pre^2 + c_gain*((t - t0_val)^2 - (t - t0_val)*T_pre)^2) / ((T_pre + t0_val - t)^2)*2*(T_pre+t0_val-t)^(-3));
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
        M_matrix = MassMatrix(q, Mlist, Glist, Slist);
        nonlinear = InverseDynamics(q, qdot, transpose(qddotDesired(i, :)), g, ...
                    zeros(6, 1), Mlist, Glist, Slist);
        if CBF_switch
            % 给定关节角度、角速度、urdf，求雅可比矩阵、非线性加速度项和末端的位置与速度
            [~, pdot1, A1, B1] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lee);
            [~, pos1] = vrep.simxGetObjectPosition(id, tool_handles, -1, vrep.simx_opmode_blocking);
            p1 = double(pos1.');
            % p1 = round(p1, 2);
            % h0 = 1*r^2 - norm(p1 - p0)^2;
            % p1_recod(:,i) = p1;
            % qddotNorm = transpose(qddotDesired(i, :)) + Kp * e + Kd * edot;
            % if h0 <= 0
                % quadprog
                t_current_ptsf = (i_ptsf-1) * dt;
                % 求约束相关参数
                h1(i) = 1*r^2 - norm(p1 - p0)^2;
                C1 = -2 * (p1 - p0).'; % g(x)
                h1_dot = C1 * pdot1;
                h2 = h1_dot + alpha1* phi(t_current_ptsf) * h1(i);
                D1 = -2*pdot1.'*pdot1 + alpha1*phi_dot(t_current_ptsf)*h1(i) + ...
                     (alpha1+alpha2)*phi(t_current_ptsf)*h1_dot + ...
                     alpha1*alpha2*phi(t_current_ptsf)^2*h1(i);
                a = -C1 * A1 / M_matrix;
                b = C1 * B1 + D1 ...
                    - C1*A1 / M_matrix * nonlinear;    
                % **求解优化问题**
                H = 2 * eye(7);               
                f = zeros(7, 1);
                [tau, fval, exitflag(i)] = quadprog(H, f, a, b, [], [], lb, ub);
                i_ptsf = i_ptsf + 1;
            % else
                % tau = M_matrix * qddotNorm + nonlinear;
                % tau = nonlinear;
            % end
            
        else
            tau = MassMatrix(q, Mlist, Glist, Slist) ...
                * (Kp * e + Kd * edot) + nonlinear;
        end
        tau = Panda_maxtorque(tau);
        tau_sim(i, :) = transpose(tau);

        % Send torque to vrep        
        for j = 1:7
            vrep.simxSetJointTargetVelocity(id, joint_handles(j), sign(tau_sim(i,j))*9999, vrep.simx_opmode_oneshot);
            vrep.simxSetJointMaxForce(id, joint_handles(j), abs(tau_sim(i,j)), vrep.simx_opmode_oneshot);
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

%% plot
figure(1)
plot(1:N,tau_sim(:,1),'r',1:N,tau_sim(:,2),'g',1:N,tau_sim(:,3),'b',1:N,tau_sim(:,4),'y',...
    1:N,tau_sim(:,5),'b',1:N,tau_sim(:,6),'c',1:N,tau_sim(:,7),'m',1:N,-40*ones(1,N),'--k','linewidth',1.5);
xlabel('Steps','fontsize',16);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('u_{1}','u_{2}','u_{3}','u_{4}','u_{5}','u_{6}','u_{7}','fontsize',15);
