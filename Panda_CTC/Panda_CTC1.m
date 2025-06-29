clear;
clc;
addpath(genpath('..\'));

%% connect to vrep
disp('Program started');
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);      % just in case, close all opened connections
id = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % connect to vrep server
if (id > -1)
    disp('Connected to remote API server');
    %%  simulaiton parameters setting
    % simulation period
    dt = 4e-3;
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
    lf = 0.107 + 0.1034;
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
    q = zeros(7, 1);
    qdot = zeros(7, 1);
    eint = zeros(7, 1);
    
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
    action1.X = Panda_syms_FK(action0.qend, l1, l2, l3, d1, d2, lf);
    action1.q7start = q7;
    action1.q7end = 0 * pi / 180;
    action1.Tf = 1;
    action1.n = action1.Tf / dt + 1;
    action1.method = 5;
    action1.q7Desired = JointTrajectory(action1.q7start, action1.q7end, ...
        action1.Tf, action1.n, action1.method);
    action1.qDesired = zeros(7, action1.n);
    for i = 1: action1.n
        action1.qDesired(:, i) = Panda_syms_IK(action1.X, ...
            l1, l2, l3, d1, d2, lf, action1.q7Desired(i), qa);
        qa = action1.qDesired(:, i);
    end
    qDesired = [qDesired; action1.qDesired'];
    N = N + action1.n;
    % stay still
    qDesired = [qDesired; ones(ns, 1) * qDesired(N, :)];
    N = N + ns;
    % action2
    action2.X = action1.X;
    action2.q7start = action1.q7end;
    action2.q7end = 90 * pi / 180;
    action2.Tf = 2;
    action2.n = action2.Tf / dt + 1;
    action2.method = 5;
    action2.q7Desired = JointTrajectory(action2.q7start, action2.q7end, ...
        action2.Tf, action2.n, action2.method);
    action2.qDesired = zeros(7, action2.n);
    for i = 1: action2.n
        action2.qDesired(:, i) = Panda_syms_IK(action2.X, ...
            l1, l2, l3, d1, d2, lf, action2.q7Desired(i), qa);
        qa = action2.qDesired(:, i);
    end
    qDesired = [qDesired; action2.qDesired'];
    N = N + action2.n;
    % stay still
    qDesired = [qDesired; ones(ns, 1) * qDesired(N, :)];
    N = N + ns;
    
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

    %% PID controller parameters
    weights = diag([1, 1, 1, 1, 1, 1, 1]);
    Kp = 30 * weights;
    Ki = 15 * weights;
    Kd = 15 * weights;
    
    %% computed torque control
    i = 1;
    while vrep.simxGetConnectionId(id) ~= -1
        if i > N
            break
        end
        % get states feedback
        for j = 1: 7
            [res, q(j)] = vrep.simxGetJointPosition(id, joint_handles(j), ...
                vrep.simx_opmode_buffer);
            [res, qdot(j)] = vrep.simxGetObjectFloatParameter(id, ...
                joint_handles(j), 2012, vrep.simx_opmode_buffer);
        end
        
        e = wrapToPi(transpose(qDesired(i, :)) - q);
        edot = transpose(qdotDesired(i, :)) - qdot;
        eint = eint + e * dt;

        q_sim(i, :) = transpose(q) * 180 / pi;
        qdot_sim(i, :) = transpose(qdot) * 180 / pi;
        e_sim(i, :) = transpose(e) * 180 / pi;
        
        nonlinear = InverseDynamics(q, qdot, transpose(qddotDesired(i, :)), g, ...
        zeros(6, 1), Mlist, Glist, Slist);

        tau = MassMatrix(q, Mlist, Glist, Slist) ...
        * (Kp * e + Kd * edot + Ki * eint) + nonlinear;
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
    % Now close the connection to vrep
    vrep.simxStopSimulation(id, vrep.simx_opmode_blocking);
    vrep.simxFinish(id);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!