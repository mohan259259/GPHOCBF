%% PTCBF6  –  SAFETY‑CBF + PD‑IN‑SPHERE  (2025‑05‑18)
%  ---------------------------------------------------------------------
%  关键要求
%    • 与原变量/函数保持一致，不改名字，不删逻辑。
%    • 进入安全球 (h ≥ 0) 后 **改用 PD**，目标关节姿态 q_center 令末端位于球心 p0。
%    • 球外 (h < 0) 继续采用 QP‑CBF。QP 不可行 → 退回 PD。
%    • 去掉 simxGetObjectMatrix，改用 simxGetObjectOrientation + 自写 eul2rotm。
%  ---------------------------------------------------------------------

clear; close all; clc;

%% ----------------------------- 连接 V‑REP ------------------------------

disp('Program started');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

% 同步模式
vrep.simxSynchronous(id,true);
if id<0, error('Failed connecting to remote API server'); end

disp('Connected to remote API server');

dt = 4e-3;
vrep.simxSetFloatingParameter(id, vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_blocking);

vrep.simxStartSimulation(id, vrep.simx_opmode_blocking);

%% --------------------------- 模型/参数 ---------------------------------

l1=0.333; l2=0.316; l3=0.384; d1=0.0825; d2=0.088; lee=0.107+0.1034;
Slist = Panda_Slist;  Mlist = Panda_Mlist;  Glist = Panda_Glist;  g=[0;0;-9.81];

vrep.simxSynchronousTrigger(id);
handles   = Panda_workcell_init(vrep,id);
joint_handles = handles.armJoints;
tool_handles  = handles.tool;
safe_region_handles = handles.safe_region;

%% ---------------------------- 初始状态 --------------------------------

q=zeros(7,1); qdot=zeros(7,1);
for j=1:7
    [~,q(j)]    = vrep.simxGetJointPosition(       id,joint_handles(j),vrep.simx_opmode_buffer);
    [~,qdot(j)] = vrep.simxGetObjectFloatParameter(id,joint_handles(j),2012,vrep.simx_opmode_buffer);
end
q_start=q; qdot_start=qdot;

%% ---------------------------- 安全球 ----------------------------------

r  = 0.12;
p0 = [0.4;0.0;0.3];
vrep.simxSetObjectPosition(id, safe_region_handles,-1, p0.', vrep.simx_opmode_oneshot);

%% --------------------- 目标关节姿态 q_center ---------------------------

% 取当前末端方向 (Euler XYZ) & 自写转换
[~,euler0] = vrep.simxGetObjectOrientation(id, tool_handles, -1, vrep.simx_opmode_blocking);
R0e = local_eul2rotm(euler0);                      % 3×3
Tdes = [R0e, p0; 0 0 0 1];

[q_center, success] = IKinSpace(Slist, Mlist(:,:,end), Tdes, q_start, 1e-4, 100);
if ~success
    warning('IK to sphere center failed – use start pose');
    q_center = q_start;
end
q_center = q_center(:);

%% --------------------------- 控制增益 ---------------------------------

N = 250;
weights = diag(ones(1,7));
Kp = 30*weights;  Kd = 15*weights;

% 只用作占位，实际球外跟踪 q_start；球内跟踪 q_center。
qDesired  = repmat(q_start', N,1);
qdotDesired = zeros(N,7);  qddotDesired = zeros(N,7);
for ii=1:N-1
    qdotDesired(ii+1,:)  = wrapToPi(qDesired(ii+1,:) - qDesired(ii,:))/dt;
    qddotDesired(ii+1,:) = (qdotDesired(ii+1,:) - qdotDesired(ii,:))/dt;
end

%% ------------------------- 日志变量预分配 -----------------------------

h1=zeros(N,1); phi1=zeros(N,1); exitflag=zeros(N,1)*0.9;
q_sim=zeros(N,7); qdot_sim=zeros(N,7); tau_sim=zeros(N,7);

%% --------------------------- CBF 参数 ---------------------------------

CBF_switch = true;
ub = 80*ones(7,1);  lb=-ub;
alpha1=2; alpha2=4;  T_pre=0.2; t0_val=0; c_gain=2;
phi=@(t) (T_pre^2 + c_gain*((t-t0_val).^2 - (t-t0_val)*T_pre).^2)./(T_pre+t0_val-t).^2;
phi_dot=@(t) (c_gain*2.*((t-t0_val).^2 - (t-t0_val)*T_pre).*(2*(t-t0_val)-T_pre).*(T_pre+t0_val-t).^2 - ...
             (T_pre^2 + c_gain*((t-t0_val).^2 - (t-t0_val)*T_pre).^2).*(-2*(T_pre+t0_val-t)))./(T_pre+t0_val-t).^4;

%% ------------------------------ 主循环 --------------------------------

q_hold = [];   % 进入球后首次记录的关节姿态

i=1; i_ptsf=1;
while vrep.simxGetConnectionId(id)~=-1
    if i>N, break; end

    % ------ 当前状态 ------------------------------------------------
    for j=1:7
        [~,q(j)]    = vrep.simxGetJointPosition(       id,joint_handles(j),vrep.simx_opmode_buffer);
        [~,qdot(j)] = vrep.simxGetObjectFloatParameter(id,joint_handles(j),2012,vrep.simx_opmode_buffer);
    end
    q=q(:); qdot=qdot(:);

    % ------ 末端位置 ------------------------------------------------
    [~,pos_eef] = vrep.simxGetObjectPosition(id, tool_handles,-1, vrep.simx_opmode_buffer);
    p = double(pos_eef.').';

    % ------ 安全函数 ------------------------------------------------
    h = r^2 - norm(p - p0)^2;  h1(i)=h;

    % ------ 动力学项 ------------------------------------------------
    M_matrix  = MassMatrix(q,Mlist,Glist,Slist);
    nonlinear = InverseDynamics(q,qdot,zeros(7,1),g,zeros(6,1),Mlist,Glist,Slist);

    % ------ 控制器 --------------------------------------------------
    % 允许略微穿过边界再交给 PD（缓解过度保守）
    if h >= 0
        % ~~~~~~~~~~~~~~~~~ 球内 – 纯 PD 保持当前位置 ~~~~~~~~~~~~~~~~~
        if isempty(q_hold)
            q_hold = q;          % 首次进入球时锁定姿态
        end
        e   = wrapToPi(q_hold - q);
        edot= -qdot;
        edot= -qdot;
        tau = M_matrix*(Kp*e + Kd*edot) + nonlinear;
        exitflag(i)=9;           % 自定义：球内
    else
        % ~~~~~~~~~~~~~~~~~ 球外 – CBF/QP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        [~,pdot1,A1,B1] = Panda_p_related(q,qdot,l1,l2,l3,d1,d2,lee);

        t_ptsf=(i_ptsf-1)*dt;
        phi_val=phi(t_ptsf);  phi_dot_val=phi_dot(t_ptsf);
        phi1(i)=phi_val;

        C1=-2*(p-p0).';  h_dot=C1*pdot1;
        D1=-2*pdot1.'*pdot1 + alpha1*phi_dot_val*h + (alpha1+alpha2)*phi_val*h_dot + alpha1*alpha2*phi_val^2*h;

        a= C1*A1/M_matrix;  b=-C1*B1 - D1 + (C1*A1/M_matrix)*nonlinear;

        H=2*eye(7); f=zeros(7,1);
        % ---- 若约束中存在 NaN/Inf 直接退回 PD ----
        if any(~isfinite(a(:))) || any(~isfinite(b(:)))
            warning('Step %d: Non‑finite QP constraints – fallback PD', i);
            tau = M_matrix*(Kp*wrapToPi(q_center - q) + Kd*(-qdot)) + nonlinear;
            exitflag(i) = -99;
        else
            [tau_qp,~,exitflag(i)] = quadprog(H,f,a,b,[],[],lb,ub);
            if isempty(tau_qp) || exitflag(i)<=0
                warning('Step %d: QP infeasible – fallback PD',i);
                tau = M_matrix*(Kp*wrapToPi(q_center - q) + Kd*(-qdot)) + nonlinear;
            else
                tau = tau_qp;
            end
        end
        i_ptsf = i_ptsf + 1;
    end

    % ------ 尺寸检查 + 饱和 ----------------------------------------
    if numel(tau)~=7||size(tau,1)~=7, error('tau size error'); end
                if isempty(tau)
                tau = nonlinear;   % fallback when tau is empty: hold position with gravity compensation
            end
            tau = Panda_maxtorque(tau);

    % ------ 记录 + 发送 --------------------------------------------
    q_sim(i,:)=q.'*180/pi;  qdot_sim(i,:)=qdot.'*180/pi;  tau_sim(i,:)=tau.';
    for j=1:7
        vrep.simxSetJointTargetVelocity(id,joint_handles(j), sign(tau(j))*9999, vrep.simx_opmode_oneshot);
        vrep.simxSetJointMaxForce(   id,joint_handles(j), abs(tau(j)),      vrep.simx_opmode_oneshot);
    end

    vrep.simxSynchronousTrigger(id); vrep.simxGetPingTime(id);
    i=i+1;
end

%% ----------------------------- 结束仿真 ------------------------------

CBFsuccess = tabulate(exitflag);

vrep.simxStopSimulation(id, vrep.simx_opmode_blocking);
vrep.simxFinish(id); vrep.delete();

%% ------------------------------ 绘图 ---------------------------------

figure(1);
plot(1:N,tau_sim(:,1),'r',1:N,tau_sim(:,2),'g',1:N,tau_sim(:,3),'b',1:N,tau_sim(:,4),'y', ...
     1:N,tau_sim(:,5),'b',1:N,tau_sim(:,6),'c',1:N,tau_sim(:,7),'m',1:N,-40*ones(1,N),'--k','linewidth',1.5);
xlabel('Steps','fontsize',16);
ylabel('$u$','interpreter','latex','fontsize',18);
legend('u_{1}','u_{2}','u_{3}','u_{4}','u_{5}','u_{6}','u_{7}','fontsize',15);

%% ------------------------- 辅助函数 ----------------------------------
function R = local_eul2rotm(eulXYZ)
% Convert XYZ Euler (rad) to rotation matrix (same as MATLAB eul2rotm(...,'XYZ'))
    cx = cos(eulXYZ(1)); sx = sin(eulXYZ(1));
    cy = cos(eulXYZ(2)); sy = sin(eulXYZ(2));
    cz = cos(eulXYZ(3)); sz = sin(eulXYZ(3));
    R = [ cy*cz,                 -cy*sz,                sy; ...
          sx*sy*cz+cx*sz,  -sx*sy*sz+cx*cz,  -sx*cy; ...
         -cx*sy*cz+sx*sz,   cx*sy*sz+sx*cz,   cx*cy ];
end

