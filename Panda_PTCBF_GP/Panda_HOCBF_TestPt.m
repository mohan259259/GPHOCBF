clear;
clc;
% addpath(genpath('..\'));

%% connect to vrep
disp('Program started');
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);      % just in case, close all opened connections
id = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % connect to vrep server
if (id > -1)
    disp('Connected to remote API server');
    %%  simulaiton parameters setting
    % simulation period
    dt = 4e-2;
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
    safe_region_handles = handles.safe_region;
    q = zeros(7, 1);
    qdot = zeros(7, 1);

    %% safe
    r = 0.06;
    % p0 = [0.53; 0; 0.2];
    [~, pos] = vrep.simxGetObjectPosition(id, safe_region_handles, -1, vrep.simx_opmode_blocking);
    p0 = double(pos.');
    GetTarget;

    %% precomputed trajectory
    N = 200;

    %% CBF parameters
    CBF_switch = true;
    alpha1 = 40;
    alpha2 = 50;
    ub = [1; 1; 1; 1; 1; 1; 1] * 80;
    lb = -ub;

    %% PD controller parameters
    weights = diag([1, 1, 1, 1, 1, 1, 1]);
    Kp = 20 * weights;
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
        
        % e = wrapToPi(transpose(qDesired(i, :)) - q);
        % edot = transpose(qdotDesired(i, :)) - qdot;

        q_sim(i, :) = transpose(q) * 180 / pi;
        qdot_sim(i, :) = transpose(qdot) * 180 / pi;
        % e_sim(i, :) = transpose(e) * 180 / pi;

        if CBF_switch
            [p1, pdot1, Jaco, Jacodot] = Panda_p_related(q, qdot, l1, l2, l3, d1, d2, lee);
            h0 = .8*r - norm(p1 - p0);
            % qddotNorm = transpose(qddotDesired(i, :)) + Kp * e + Kd * edot;
            h1(i) = (1*r)^2 - norm(p1 - p0)^2;
            C1 = transpose(p1 - p0);
            phi1(i) = 2 * C1 * pdot1 + alpha1 * h1(i);
            if h0 <= 0
                % quadprog
                H = diag([2, 2, 2, 2, 2, 2, 2]);
                f = zeros(7, 1);
                fx = norm(pdot1)^2 + (alpha1 + alpha2)/2 * phi1(i) - alpha1^2/2 * h1(i);
                A = C1 * Jaco;
                Aineq = -A;
                b = -(C1 * Jacodot + fx);
                bineq = -b;
                [qddotSafe, fval, exitflag(i)] = quadprog(H, f, Aineq, bineq, [], [], lb, ub);
                % fmincon
                % fx = norm(pdot1)^2 + (alpha1 + alpha2)/2 * phi1(i) - alpha1^2/2 * h1(i);
                % a = -C1 * Jaco;
                % b = (C1 * Jacodot + fx);            
                % 定义目标函数
                % objective = @(qddot) qddot;            
                % 定义约束函数
                % constraint = @(qddot) deal([], a * qddot - b); % 不等式约束：a * qddot <= b            
                % 初始猜测
                % qddot0 = zeros(size(7, 1), 1);            
                % 设置 fmincon 选项
                % options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'interior-point');            
                % 调用 fmincon 求解
                % [qddotSafe, fval, exitflag, output] = fmincon(objective, qddot0, [], [], [], [], lb, ub, constraint, options);                
            else
                % qddotSafe = qddotNorm;
            end
            if exitflag(i) == -2
                qddotSafe = zeros(7,1);
            end
            tau = MassMatrix(q, Mlist, Glist, Slist) * qddotSafe ...
                + InverseDynamics(q, qdot, zeros(7, 1), g, zeros(6, 1), Mlist, Glist, Slist);
        else
            % nonlinear = InverseDynamics(q, qdot, transpose(qddotDesired(i, :)), g, ...
            %     zeros(6, 1), Mlist, Glist, Slist);
            % tau = MassMatrix(q, Mlist, Glist, Slist) ...
            %     * (Kp * e + Kd * edot) + nonlinear;
            disp("please open cbf");
            break;
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
    % CBFsuccess = tabulate(exitflag);
    % Now close the connection to vrep
    vrep.simxStopSimulation(id, vrep.simx_opmode_blocking);
    vrep.simxFinish(id);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!
%%
figure(1)
plot(1:N,tau_sim(:,1),'r',1:N,tau_sim(:,2),'g',1:N,tau_sim(:,3),'b',1:N,tau_sim(:,4),'y',...
    1:N,tau_sim(:,5),'b',1:N,tau_sim(:,6),'c',1:N,tau_sim(:,7),'m',1:N,-40*ones(1,N),'--k','linewidth',1.5);
xlabel('Steps','fontsize',16);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('u_{1}','u_{2}','u_{3}','u_{4}','u_{5}','u_{6}','u_{7}','fontsize',15);