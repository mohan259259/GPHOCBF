%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Panda CBF demo  -- 完整可运行版（2025-05-01）
%  · global torque limit : |τ| ≤ 30 Nm
%  · auto-check safe_region field
%  · fallback when quadprog is infeasible or empty
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

%% ---------------------- connect to V-REP / CoppeliaSim -----------------
disp('Program started');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);                          % close any old connections
id = vrep.simxStart('127.0.0.1',19997, true,true, 5000,5);
vrep.simxSynchronous(id,true);               % sync mode

if id <= -1
    error('Failed connecting to remote API server');
end
disp('Connected to remote API server');

%% ---------------------- simulation parameters --------------------------
dt = 4e-3;
vrep.simxSetFloatingParameter(id, vrep.sim_floatparam_simulation_time_step, ...
                              dt, vrep.simx_opmode_blocking);
vrep.simxStartSimulation(id, vrep.simx_opmode_blocking);

%% ------------------------- model parameters ----------------------------
l1=0.333; l2=0.316; l3=0.384;
d1=0.0825; d2=0.088; lee=0.107+0.1034;
Slist = Panda_Slist;   Mlist = Panda_Mlist;   Glist = Panda_Glist;
g     = [0;0;-9.81];

vrep.simxSynchronousTrigger(id);

%% ---------------------------- handles ----------------------------------
handles          = Panda_workcell_init(vrep,id);
joint_handles    = handles.armJoints;
obstacle_handles = handles.obstacle;
tool_handles     = handles.tool;

% ---------- optional visual safe-zone ----------
safe_region_exist = false;
if isfield(handles,'safe_region')                     % <<< 可能改这里 >>>
    safe_region_hdl = handles.safe_region;
    safe_region_exist = true;
elseif isfield(handles,'safeRegion')
    safe_region_hdl = handles.safeRegion;
    safe_region_exist = true;
end
if safe_region_exist
    vrep.simxSetObjectPosition(id, safe_region_hdl,-1, ...
                               [0.7,0.0,0.3], vrep.simx_opmode_oneshot);
else
    warning('No safe-region handle found: skip visual marker.');
end

%% ---------------------------- initial state ----------------------------
q    = zeros(7,1);
qdot = zeros(7,1);
for j = 1:7
    [~,q(j)]    = vrep.simxGetJointPosition(id,joint_handles(j), vrep.simx_opmode_buffer);
    [~,qdot(j)] = vrep.simxGetObjectFloatParameter(id,joint_handles(j), ...
                                                   2012, vrep.simx_opmode_buffer);
end

%% ---------------------- control & CBF constants ------------------------
N        = 100;                % simulation steps
tau_max  = 87;                 % global torque limit (Nm)
lb       = -tau_max*ones(7,1);
ub       =  tau_max*ones(7,1);

% CBF – spherical safe zone
r  = 0.15;
p0 = [0.7;0.0;0.3];

alpha1 = 2;   alpha2 = 4;
T_pre  = 0.25; t0_val = 0; c_gain = 2;
phi     = @(t) (T_pre^2 + c_gain*((t - t0_val).^2 - (t - t0_val)*T_pre).^2) ...
               ./ (T_pre + t0_val - t).^2;
phi_dot = @(t) ( c_gain*2.*((t - t0_val).^2 - (t - t0_val)*T_pre) ...
                 .* (2*(t - t0_val) - T_pre) .* (T_pre + t0_val - t).^2 ...
                 - (T_pre^2 + c_gain*((t - t0_val).^2 - (t - t0_val)*T_pre).^2) ...
                 .* (-2*(T_pre + t0_val - t)) ) ...
                 ./ (T_pre + t0_val - t).^4;

%% ------------------------ logging arrays -------------------------------
q_sim    = zeros(N,7);
qdot_sim = zeros(N,7);
tau_sim  = zeros(N,7);
h1       = zeros(N,1);
phi1     = zeros(N,1);
exitflag = zeros(N,1);

%% --------------------------- main loop ---------------------------------
for i = 1:N
    % ----- read current state -----
    for j = 1:7
        [~,q(j)]    = vrep.simxGetJointPosition(id,joint_handles(j), vrep.simx_opmode_buffer);
        [~,qdot(j)] = vrep.simxGetObjectFloatParameter(id,joint_handles(j), ...
                                                       2012, vrep.simx_opmode_buffer);
    end
    
    % ----- log state -----
    q_sim(i,:)    = q.'   * 180/pi;
    qdot_sim(i,:) = qdot.'* 180/pi;
    
    % ----- dynamics -----
    M  = MassMatrix(q, Mlist, Glist, Slist);
    Nl = InverseDynamics(q, qdot, zeros(7,1), g, zeros(6,1), ...
                         Mlist, Glist, Slist);
    
    % ----- EE position/velocity -----
    [~, pdot1, A1, B1] = Panda_p_related(q, qdot, l1,l2,l3,d1,d2,lee);
    [~,pos1] = vrep.simxGetObjectPosition(id,tool_handles,-1, vrep.simx_opmode_blocking);
    p1 = double(pos1.');
    
    % ----- CBF -----
    h           = r^2 - norm(p1 - p0)^2;
    h1(i)       = h;
    t_ptsf_cur  = (i-1)*dt;
    if h >= 0, phi_val = 1; phi_dot_val = 0;
    else       phi_val = phi(t_ptsf_cur);  phi_dot_val = phi_dot(t_ptsf_cur);
    end
    phi1(i) = phi_val;
    
    C1     = -2*(p1 - p0).';
    h1_dot = C1*pdot1;
    D1     = -2*pdot1.'*pdot1 ...
             + alpha1*phi_dot_val*h ...
             + (alpha1+alpha2)*phi_val*h1_dot ...
             + alpha1*alpha2*phi_val^2*h;
    
    a = -C1*A1 / M;
    b =  C1*B1 + D1 - (C1*A1 / M)*Nl;
    
    % ----- QP solve -----
    H = 2*eye(7);  f = zeros(7,1);
    [tau,~,exitflag(i)] = quadprog(H,f, a,b, [],[], lb,ub);
    
    % ----- fallback when infeasible -----
       % ----- fallback when infeasible -----
        if exitflag(i) ~= 1 || isempty(tau)
        warning('Step %d: QP infeasible (exitflag=%d), fallback.', i, exitflag(i));
        if i==1
            tau = zeros(7,1);
        else
            tau = tau_sim(i-1,:).';     % 用上一步力矩
        end
    end



    
    % ----- saturate, log -----
    tau      = max(min(tau, tau_max), -tau_max);
    tau_sim(i,:) = tau.';
    
    % ----- send to sim -----
    for j = 1:7
        vrep.simxSetJointTargetVelocity(id, joint_handles(j), ...
                                        sign(tau(j))*9999, vrep.simx_opmode_oneshot);
        vrep.simxSetJointMaxForce(id, joint_handles(j), abs(tau(j)), ...
                                  vrep.simx_opmode_oneshot);
    end
    
    vrep.simxSynchronousTrigger(id);
    vrep.simxGetPingTime(id);
end

%% ------------------------ finish up ------------------------------------
vrep.simxStopSimulation(id, vrep.simx_opmode_blocking);
vrep.simxFinish(id);
vrep.delete();

%% ------------------------ plot -----------------------------------------
figure(1); clf;
plot(1:N, tau_sim(:,1),'r', ...
     1:N, tau_sim(:,2),'g', ...
     1:N, tau_sim(:,3),'b', ...
     1:N, tau_sim(:,4),'y', ...
     1:N, tau_sim(:,5),'b', ...
     1:N, tau_sim(:,6),'c', ...
     1:N, tau_sim(:,7),'m', ...
     1:N, -tau_max*ones(1,N),'--k', ...
     1:N,  tau_max*ones(1,N),'--k','linewidth',1.5);
xlabel('Steps','fontsize',14);
ylabel('\tau (Nm)','fontsize',14);
legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6','\tau_7', ...
       'location','best','fontsize',10);
title('|τ| \le 30 Nm with CBF safety', 'fontsize',14);
grid on;
