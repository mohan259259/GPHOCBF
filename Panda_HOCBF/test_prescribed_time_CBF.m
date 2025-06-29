function test_prescribed_time_CBF
    clc; clear; close all;
    
    % =========== 用户/问题设置 ===========

    % 1) 安全球: 半径 r, 球心 p0(3x1)
    r    = 1.0;
    p0   = [0;0;0];

    % 2) 预设时间参数
    T_s     = 2;          % safety “deadline”
    T_theta = 1;
    T_psi   = (T_s + T_theta)/2;  
    c_gain  = 1.0;
    t0_val  = 0.0;  

    % 3) CBF控制参数
    alpha0  = 1.0;
    alpha1  = 1.0;
    beta    = 1.0;
    gamma   = 1.0;

    % 4) 名义PD增益 (3×3)
    Kp      = 10.0*eye(3);
    Kd      = 5.0*eye(3);

    % 5) “GP” mean & std. dev. (mocked)
    mu_fn   = @(x,theta_hat) ones(3,1); % 3×1
    sigma_fn= @(x,theta_hat) 0.5;       % scalar

    % 6) PD 期望位置 (q_des in 3D)
    q_des   = [3; 3; 3];

    % 7) 初始状态: q(0)=[3;3;3], dq(0)=[0;0;0] (在球外)
    q0      = [3; 3; 3];
    dq0     = [0; 0; 0];
    x0      = [q0; dq0];  % (6×1)

    % 8) 积分器设置
    opts    = odeset('RelTol',1e-7,'AbsTol',1e-8);

    % ========== 第一段积分： [0, 1.99] ==========
    tspan1   = [0, 1.99]; 
    [tSol1, xSol1] = ode45(@(t,x) robotDynamics(t,x), tspan1, x0, opts);
    xEnd_1 = xSol1(end,:);

    % ========== 第二段积分： [2.01, 10], 跳过 t=2 ==========
    tspan2  = [2.01, 10];
    [tSol2, xSol2] = ode45(@(t,x) robotDynamics(t,x), tspan2, xEnd_1, opts);

    % ========== 合并两段结果 ==========
    tSol = [tSol1; tSol2];
    xSol = [xSol1; xSol2];

    % ========== 画图: h(q(t)) vs. time ==========
    figure('Position',[100,100,1200,600]);
    hold on; grid on;
    hVals = arrayfun(@(i) hFun(xSol(i,1:3).'), 1:length(tSol));
    plot(tSol, hVals, 'LineWidth',2,'DisplayName','h(q(t))');
    plot([T_s T_s], ylim, 'r--','DisplayName','t = T_s');
    xlabel('Time (s)');  ylabel('h(q(t))');
    title('Barrier function vs time (3D)');
    legend('Location','Best');

    disp('===========================');
    disp(['Final time = ', num2str(tSol(end)), ...
          ',  h(q(tEnd)) = ', num2str(hVals(end))]);
    disp(['Check near T_s=2 =>  index where t>=2: ', ...
          num2str(find(tSol>=2,1))]);
    disp('===========================');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% 下面是嵌套函数: 动力学, CBF控制律, blow-up计算等 %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % ========== 机器人动力学: x=[q; dq], ddq=tau, M=I(3) ==========
    function dxdt = robotDynamics(t,x)
        % x(1:3)=q, x(4:6)=dq
        q  = x(1:3);
        dq = x(4:6);

        % 调用 controlLaw 得到 tau(3×1)
        tau = controlLaw(t, x);

        % ddq = tau => dxdt=[dq; ddq]
        ddq = tau;
        dxdt = zeros(6,1);
        dxdt(1:3) = dq;
        dxdt(4:6) = ddq;
    end

    % ========== 控制律 (59) ==========
    function tau = controlLaw(t,x)
        q  = x(1:3);
        dq = x(4:6);

        % 计算 a_mu
        psi_val   = psiFun(t,q,dq);
        ah_val    = a_hFun(t,q,dq);
        bh_val    = b_hFun(q);  % 3×1

        mu_val    = mu_fn(x,[]);
        sigma_val = sigma_fn(x,[]);
        amu_val   = ah_val + bh_val'*mu_val ...
                    - norm(bh_val)*(beta*sigma_val + gamma) ...
                    + z1Fun(t)*alpha1*psi_val;

        % 名义PD
        tau_n = -Kp*(q - q_des) - Kd*dq;  % 3×1

        lhs_check = amu_val + bh_val'*tau_n;
        if lhs_check >= 0
            tau = tau_n;
        else
            denom = (bh_val'*bh_val);
            I3    = eye(3);
            proj  = I3 - (bh_val*bh_val')/denom;
            tau_s = proj*tau_n - (amu_val/denom)*bh_val;
            tau   = tau_s;
        end
    end

    % ========== barrier function: h(q)= r^2 - ||q - p0||^2 ==========
    function val = hFun(q_3)
        val = r^2 - norm(q_3 - p0)^2;
    end

    % ========== psiFun(t,q,dq)= dh/dq(q)*dq + alpha0*z0Fun*h(q) ==========
    function val = psiFun(t,q,dq)
        % dhdqFun(q) 是 3×1 => dhdqFun(q)' * dq => 标量
        val = dhdqFun(q)'*dq + alpha0*z0Fun(t)*hFun(q);
    end

    % ========== a_hFun(t,q,dq) (57), 忽略C,g=0 ==========
    function val = a_hFun(t,q,dq)
        % a_h= dq^T( d2h_dq2 )dq + alpha0*z0*(dhdqFun(q)'*dq) + alpha0*dz0*h(q)
        H_   = d2hdq2Fun(q);       % 3×3
        term1= dq.'*H_*dq;         % 标量
        term2= alpha0*z0Fun(t)*( dhdqFun(q)'*dq );
        term3= alpha0*dz0Fun(t)* hFun(q);
        val  = term1 + term2 + term3;
    end

    % ========== b_hFun= dhdqFun(q), M=I => 3×1 ==========
    function bh = b_hFun(q)
        bh = dhdqFun(q);
    end

    % ========== dhdqFun(q)= -2(q - p0), 3×1 ==========
    function grad_h_3 = dhdqFun(q_3)
        grad_h_3 = -2*(q_3 - p0);
    end

    % ========== d2hdq2Fun(q)= Hessian(3×3)= -2I(3) ==========
    function H_3x3 = d2hdq2Fun(~)
        H_3x3 = -2*eye(3);
    end

    % ========== blow-up 函数 z0, dz0, z1 ==========
    function val = z0Fun(t)
        numerator   = (T_s^2 + c_gain*(((t - t0_val)^2 - (t-t0_val)*T_s)^2));
        denominator = (T_s - t)^2;
        val = numerator / denominator;
    end

    function val = dz0Fun(t)
        A = (t - t0_val);
        B = (T_s - t);
        part1 = c_gain*2*(A^2 - A*T_s)*(2*A - T_s)*B^2;
        part2 = (T_s^2 + c_gain*((A^2 - A*T_s)^2))*(-2*B);
        val   = (part1 - part2)/(B^4);
    end

    function val = z1Fun(t)
        numerator   = (T_psi^2 + c_gain*(((t - t0_val)^2 - (t-t0_val)*T_psi)^2));
        denominator = (T_psi - t)^2;
        val = numerator / denominator;
    end
end
