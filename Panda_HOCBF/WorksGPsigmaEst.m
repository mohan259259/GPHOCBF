clear; clc;
%% connect to vrep
disp('Program started');
vrep = remApi('remoteApi');                      % using the prototype file
vrep.simxFinish(-1);                             % just in case, close all connections
id = vrep.simxStart('127.0.0.1',19997,true,true,5000,5); % connect to vrep
if (id > -1)
    disp('Connected to remote API server');

    %%  simulation parameters
    dt = 4e-3;
    vrep.simxSetFloatingParameter(id,vrep.sim_floatparam_simulation_time_step,...
                                  dt,vrep.simx_opmode_blocking);
    vrep.simxSynchronous(id,true);
    vrep.simxStartSimulation(id,vrep.simx_opmode_blocking);

    %%  initialization
    l1 = 0.333;  l2 = 0.316;  l3 = 0.384;
    d1 = 0.0825; d2 = 0.088;  lee = 0.107 + 0.1034;
    Slist = Panda_Slist;  Mlist = Panda_Mlist;  Glist = Panda_Glist;
    g = [0;0;-9.81];
    vrep.simxSynchronousTrigger(id);
    handles = Panda_workcell_init(vrep,id);
    joint_handles = handles.armJoints;
    q = zeros(7,1);  qdot = zeros(7,1);

    %% obstacle (safety-ball)
    r = 0.05;      p0 = [0.55;0.00;0.35];
    N = 620;

    h1 = zeros(N,1);  phi1 = zeros(N,1);  exitflag = ones(N,1)*0.9;
    q_sim = zeros(N,7);  qdot_sim = zeros(N,7);  tau_sim = zeros(N,7);p1_traj     = zeros(N, 3);      % <<< NEW：记录末端位置 [x y z]

    %% CBF parameters
    CBF_switch = 1;  alpha1 = 5;  alpha2 = 5;
    ub = ones(7,1)*80;  lb = -ub;
    omega = 1;  t_d = 1.001;

    %% PD gains
    weights = diag(ones(1,7));
    Kp = 30*weights;  Kd = 15*weights;

    %%% --------------------  GP INITIALISATION  -------------------- %%%
    gp = LocalGP_MultiOutput(14,7,300,0.1,0.51,-0.48);   %%%% <<< NEW
    train_every = 5;                                 %%%% <<< NEW
    %%% -------------------------------------------------------------- %%%

    %% main loop
    i = 1;
    while vrep.simxGetConnectionId(id) ~= -1
        if i > N, break, end

        % feedback
        for j = 1:7
            [~,q(j)]   = vrep.simxGetJointPosition(id,joint_handles(j),vrep.simx_opmode_buffer);
            [~,qdot(j)] = vrep.simxGetObjectFloatParameter(id,joint_handles(j),2012,...
                                                           vrep.simx_opmode_buffer);
        end
        q_sim(i,:)   = q'*180/pi;
        qdot_sim(i,:) = qdot'*180/pi;

        xi(i)     = exp(omega*(t_d-i*dt)) - 1;
        xidot(i)  = -omega*exp(omega*(t_d-i*dt));
        xiddot(i) = omega^2*exp(omega*(t_d-i*dt));

        if CBF_switch
            [p1,pdot1,A1,B1] = Panda_p_related(q,qdot,l1,l2,l3,d1,d2,lee);
            p1_traj(i, :) = p1';            % <<< NEW：存当前末端位置
            h1(i) = r^2 - norm(p1-p0)^2;
            C1 = (p1-p0)';
            if h1(i) < 0
                k = h1(i);
                k2 = abs(xidot(i))/xi(i)*-2*C1*pdot1;
            else
                k = 0; k2 = 0;
            end
            phi1(i) = -2*C1*pdot1 + alpha1*h1(i) + abs(xidot(i))/xi(i)*k;
            H = diag(2*ones(1,7));  f = zeros(7,1);
            D1 = -2*norm(pdot1)^2 + alpha1*(-2*C1*pdot1) ...
                - (xiddot(i)-xidot(i)^2)/xi(i)^2 * k + k2 + alpha2*phi1(i);
            a = 2*C1*A1;   b = -2*C1*B1 + D1;
            [qddotSafe,~,exitflag(i)] = quadprog(H,f,a,b,[],[],lb,ub);

            tau = MassMatrix(q,Mlist,Glist,Slist)*qddotSafe ...
                + InverseDynamics(q,qdot,zeros(7,1),g,zeros(6,1),Mlist,Glist,Slist);
        else
            % (此处省略 else 分支内容，因 CBF_switch 始终为 1)
        end




        % ---------- 1) 计算当前状态 ----------
x = [q; qdot];                       % 14×1

% ---------- 2) 真实扰动 d(x) ----------
d = 0.03*sin(x(1:7)) + 0.03*cos(x(8:14));  % 7×1
% **不** 再把 d 加进 τ
        % ---------- 用 GP 方差补偿  τ ← τ + βσ + γ ----------
beta  = 0.51;            % β 系数
gamma = 0.0004;            % γ 常数项

x = [q; qdot];        % 当前 14 维状态

if gp.DataQuantity > 0
    [~, sigma, ~, ~, ~, ~, ~] = gp.predict(x);   % 7×1 预测方差 σ(x)
else
    sigma = zeros(7,1);
end

tau = tau + beta * sigma + gamma;   % 最终补偿
% ----------------------------------------------------
                            % 补偿 GP 预测值
        % --------------- GP 训练 (加点 / 删点) ---------------
        if mod(i,train_every) == 0
            if gp.DataQuantity >= gp.MaxDataQuantity
                gp.downdateParam(1);               % 删最旧样本
            end
            gp.addPoint(x,d);                      % 加新样本 (x,d)
        end
        % ----------------------------------------------------

        tau = Panda_maxtorque(tau);
        tau_sim(i,:) = tau';

        % send torque
        for j = 1:7
            if tau(j) < 0
                set_vel = -99999; set_tau = -tau(j);
            else
                set_vel = 99999;  set_tau = tau(j);
            end
            vrep.simxSetJointTargetVelocity(id,joint_handles(j),set_vel,...
                                            vrep.simx_opmode_oneshot);
            vrep.simxSetJointMaxForce(id,joint_handles(j),set_tau,...
                                      vrep.simx_opmode_oneshot);
        end
        vrep.simxSynchronousTrigger(id);
        vrep.simxGetPingTime(id);
        i = i + 1;
    end

    CBFsuccess = tabulate(exitflag);
    vrep.simxStopSimulation(id,vrep.simx_opmode_blocking);
    vrep.simxFinish(id);
else
    disp('Failed connecting to remote API server');
end
vrep.delete();   % call the destructor