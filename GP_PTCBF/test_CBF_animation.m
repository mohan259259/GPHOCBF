function test_CBF_animation
    % 清除环境
    clc; clear; close all;

    % 声明全局变量，用于记录
    global tau_nom_history lhs_check_history control_time_history control_counter;
    tau_nom_history      = [];
    lhs_check_history    = [];
    control_time_history = [];
    control_counter      = 0;

    %% ========== 用户/问题设置 ==========
    % Barrier 半径与中心
    r    = 1.0;
    p0   = [0;0;0];

    % 预设时间参数
    T_s     = 2;          
    T_theta = 1;          
    T_psi   = (T_s+T_theta)/2;
    c_gain  = 1.0;
    t0_val  = 0.0;

    % CBF 增益
    alpha0 = 1;  alpha1 = 1;
    beta   = 1;  gamma  = 1;

    % PD 名义控制律增益
    Kp = 10*eye(3);
    Kd =  5*eye(3);

    % 模拟 GP 的均值和方差函数
    mu_fn    = @(x,theta_hat) ones(3,1);
    sigma_fn = @(x,theta_hat) 0.5;

    % 期望位置
    q_des = [3;3;3];

    % 初始状态
    q0  = [3;3;3];
    dq0 = [0;0;0];
    x0  = [q0; dq0];

    % ode45 选项
    opts = odeset('RelTol',1e-7,'AbsTol',1e-8);

    %% ========== 分段积分 ==========
    % 第一段 0→1.49
    [t1,x1] = ode45(@robotDynamics, [0,1.49],  x0, opts);
    % 第二段 1.51→1.99
    [t2,x2] = ode45(@robotDynamics, [1.51,1.99], x1(end,:), opts);
    % 第三段 2.01→10
    [t3,x3] = ode45(@robotDynamics, [2.01,10],   x2(end,:), opts);

    % 合并
    tSol = [t1; t2; t3];
    xSol = [x1; x2; x3];

    % 提取末端位置轨迹
    qTraj = xSol(:,1:3).';  % 3×N

    %% ========== 打印初末位置 ==========
    fprintf('At t=0, q = [%.4f,%.4f,%.4f]\n', qTraj(:,1));
    fprintf('At t=10, q = [%.4f,%.4f,%.4f]\n\n', qTraj(:,end));

    %% ========== 动画展示 ==========
    figure('Position',[200,200,800,600]);
    hold on; axis equal; grid on
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(35,20);
    title('末端轨迹动态动画');

    % 画安全球体
    [X,Y,Z] = sphere(50);
    surf(X, Y, Z, 'FaceAlpha',0.1,'EdgeColor','none');
    colormap jet;

    % 准备动画线条
    hAnim = animatedline('LineWidth',2,'Marker','o','MarkerSize',5,'MarkerFaceColor','r');
    legend('Trajectory','Safety Zone','Location','best');

    % 以平均步长 pause
    dt_mean = mean(diff(tSol));
    for k = 1:length(tSol)
        addpoints(hAnim, qTraj(1,k), qTraj(2,k), qTraj(3,k));
        drawnow;
        pause(dt_mean);
    end

    %% ========== （可选）回传至 base workspace ==========
    assignin('base','tau_nom_history',       tau_nom_history);
    assignin('base','lhs_check_history',    lhs_check_history);
    assignin('base','control_time_history', control_time_history);
    disp('控制记录已存入 workspace.');

    %% ======== 嵌套函数：动力学 + 控制律 + CBF 辅助函数 ========
    function dxdt = robotDynamics(t,x)
        q  = x(1:3);
        dq = x(4:6);
        tau = controlLaw(t,x);
        % 简化模型 M=I => ddq = tau
        dxdt = [dq; tau];
    end

    function tau = controlLaw(t,x)
        q  = x(1:3);
        dq = x(4:6);

        % CBF 项
        h_val   = r^2 - norm(q-p0)^2;
        dhdq    = -2*(q-p0);
        psi_val = dhdq.'*dq + alpha0*z0(t)*h_val;
        ah_val  = dq.'*(-2*eye(3))*dq ...
                  + alpha0*z0(t)*(dhdq.'*dq) ...
                  + alpha0*dz0(t)*h_val;
        bh_val  = dhdq;

        % GP 估计
        mu_val    = mu_fn(x,[]);
        sigma_val = sigma_fn(x,[]);

        % 安全修正量
        amu = ah_val + bh_val.'*mu_val ...
               - norm(bh_val)*(beta*sigma_val+gamma) ...
               + alpha1*z1(t)*psi_val;

        % nominal PD 控制
        tau_n = -Kp*(q-q_des) - Kd*dq;

        % 判别式
        lhs = amu + bh_val.'*tau_n;

        % 记录
        control_counter = control_counter + 1;
        control_time_history(control_counter) = t;
        tau_nom_history(:,control_counter)     = tau_n;
        lhs_check_history(control_counter)     = lhs;

        % 选取
        if lhs >= 0
            tau = tau_n;
        else
            % 投影 + 修正
            denom = bh_val.'*bh_val;
            tau = (eye(3)-bh_val*bh_val.'/denom)*tau_n ...
                  - (amu/denom)*bh_val;
        end
    end

    % z0, dz0, z1
    function v = z0(t)
        A = (t-t0_val); B = (T_s-t);
        v = (T_s^2 + c_gain*(A^2 - A*T_s)^2)/(B^2);
    end
    function v = dz0(t)
        A = (t-t0_val); B = (T_s-t);
        part1 = c_gain*2*(A^2-A*T_s)*(2*A-T_s)*B^2;
        part2 = (T_s^2 + c_gain*(A^2-A*T_s)^2)*(-2*B);
        v = (part1-part2)/(B^4);
    end
    function v = z1(t)
        A = (t-t0_val); B = (T_psi-t);
        v = (T_psi^2 + c_gain*(A^2 - A*T_psi)^2)/(B^2);
    end

end
