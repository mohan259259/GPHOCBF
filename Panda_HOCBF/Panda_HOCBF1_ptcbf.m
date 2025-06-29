clear;
clc;
%addpath(genpath('..\'));

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
    r = 0.05;
    p0 = [0.53; 0; 0.2];
    N=550;
    
    h1 = zeros(N, 1);
    phi1 = zeros(N, 1);
    exitflag = ones(N, 1) * 0.9;
    qdotDesired = zeros(N, 7);
    qddotDesired = zeros(N, 7);
    q_sim = zeros(N, 7);
    qdot_sim = zeros(N, 7);
    e_sim = zeros(N, 7);
    tau_sim = zeros(N, 7);

%     for i = 1: N - 1
%         qdotDesired(i + 1, :) = wrapToPi(qDesired(i + 1, :) - qDesired(i, :)) / dt;
%         qddotDesired(i + 1, :) = (qdotDesired(i + 1, :) - qdotDesired(i, :)) / dt;
%     end

    %% CBF parameters
    CBF_switch = 1;
    alpha1 = 5;
    alpha2 = 5;
    %ub = [87; 87; 87; 87; 12; 12; 12];
    ub = [1; 1; 1; 1; 1; 1; 1]*80;
    lb = -ub;
    omega=1;
    t_d=1.001;
     
    

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
        
%         e = wrapToPi(transpose(qDesired(i, :)) - q);
%         edot = transpose(qdotDesired(i, :)) - qdot;

        q_sim(i, :) = transpose(q) * 180 / pi;
        qdot_sim(i, :) = transpose(qdot) * 180 / pi;
        %e_sim(i, :) = transpose(e) * 180 / pi;
        
        xi(i)=exp(omega*(t_d-i*dt))-1;
        xidot(i)=-omega*exp(omega*(t_d-i*dt));
        xiddot(i)=omega^2*exp(omega*(t_d-i*dt));

        if CBF_switch
            [p1, pdot1, A1, B1] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lee);  %pdot1:J*qdot; A1:J; B1:Jdot*qdot
%             h0 = norm(p1 - p0)^2 - (5*r)^2;
%             qddotNorm = transpose(qddotDesired(i, :)) + Kp * e + Kd * edot;
            h1(i) = r^2 - norm(p1 - p0)^2;
            C1 = transpose(p1 - p0);
            if h1(i)<0
                k=h1(i);
                k2=abs(xidot(i))/xi(i)*-2 * C1 * pdot1;
            else
                k=0;
                k2=0;
            end
            phi1(i) = -2 * C1 * pdot1 + alpha1 * h1(i)+abs(xidot(i))/xi(i)*k;
            %if h0 <= 0
                % quadprog
                H = diag([2, 2, 2, 2, 2, 2, 2]);
                %f = -2 * qddotNorm;
                f=zeros(7,1);
                %D1 = norm(pdot1)^2 + (alpha1 + alpha2)/2 * phi1(i) - alpha1^2/2 * h1(i);
                D1=-2*norm(pdot1)^2 + alpha1 * -2 * C1 * pdot1 - (xiddot(i)-xidot(i)^2)/xi(i)/xi(i)*k + k2 + alpha2*phi1(i);
                a = 2 * C1 * A1;
                b = -2 * C1 * B1 + D1;
                [qddotSafe, fval, exitflag(i)] = quadprog(H, f, a, b, [], [], lb, ub);
            %else
                %qddotSafe = qddotNorm;
            %end
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