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
    lf = 0.107 + 0.05;
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
    q = zeros(7, 1);
    qdot = zeros(7, 1);

    %% obstacle
    r = 0.03;
    p0 = [0.53; 0; 0.2];
    
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
    action1.Rend = action1.Rstart * MatrixExp3(VecToso3(omghat_z ...
        * (0 * pi / 180)));
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
    action2.Rend = action2.Rstart * MatrixExp3(VecToso3(omghat_z ...
        * (0 * pi / 180)));
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
    h2 = zeros(N, 1);
    phi1 = zeros(N, 1);
    phi2 = zeros(N, 1);
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
    alpha1 = 5;
    alpha2 = 30;
    ub = [1; 1; 1; 1; 1; 1; 1] * 80;
    lb = -ub;

    %% PD controller parameters
    weights = diag([1, 1, 1, 1, 1, 1, 1]);
    Kp = 30 * weights;
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

        q_sim(i, :) = transpose(q) * 180 / pi;
        qdot_sim(i, :) = transpose(qdot) * 180 / pi;
        e_sim(i, :) = transpose(e) * 180 / pi;

        if CBF_switch
            [p1, pdot1, A1, B1] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lee);
            [p2, pdot2, A2, B2] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lf);
            h0 = norm(p1 - p0)^2 - (5*r)^2;
            % h0 = 0;
            qddotNorm = transpose(qddotDesired(i, :)) + Kp * e + Kd * edot;
            h1(i) = norm(p1 - p0)^2 - r^2;
            h2(i) = norm(p2 - p0)^2 - r^2;
            C1 = transpose(p1 - p0);
            C2 = transpose(p2 - p0);
            phi1(i) = 2 * C1 * pdot1 + alpha1 * h1(i);
            phi2(i) = 2 * C2 * pdot2 + alpha1 * h2(i);
            if h0 <= 0
                % quadprog
                H = diag([2, 2, 2, 2, 2, 2, 2]);
                f = -2 * qddotNorm;
                D1 = norm(pdot1)^2 + (alpha1 + alpha2)/2 * phi1(i) - alpha1^2/2 * h1(i);
                D2 = norm(pdot2)^2 + (alpha1 + alpha2)/2 * phi2(i) - alpha1^2/2 * h2(i);
                a = [-C1 * A1;
                     -C2 * A2];
                b = [C1 * B1 + D1;
                     C2 * B2 + D2];
                [qddotSafe, fval, exitflag(i)] = quadprog(H, f, a, b, [], [], lb, ub);
            else
                qddotSafe = qddotNorm;
            end
            tau = MassMatrix(q, Mlist, Glist, Slist) * qddotSafe ...
                + InverseDynamics(q, qdot, zeros(7, 1), g, zeros(6, 1), Mlist, Glist, Slist);
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