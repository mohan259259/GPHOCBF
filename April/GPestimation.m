function GPestimation
% =========================================
% 1-DOF 同时估计 beta_hat & theta_hat
% 图中额外显示“最终估计值”说明框
% =========================================

%% 0) 名义值交互输入
prompt = {'beta_hat_nominal:', 'GP sigma_f_nominal:', 'GP l_nominal:'};
ansDlg = inputdlg(prompt,'Nominal values',[1 50],{'1.0','1.0','1.0'});
if isempty(ansDlg);  disp('canceled');  return;  end
beta_hat = str2double(ansDlg{1});
theta_n  = [str2double(ansDlg{2}); str2double(ansDlg{3})];

%% 1) 系统与控制参数
M = 1;  k = 0.1;  m = 1;  g = 9.8;
Cfun = @(dq) k*dq.^2;
gfun = @(q) m*g*sin(q);
d_true = @(x) sin(x(1)) + cos(x(2));
p0 = 0;  r = 1.5;  Kp = 10;  Kd = 5;

beta_true = 1.0;
theta_true = [0.5; 1.5];

%% 2) GP 对象（接口需匹配你的 LocalGP_MultiOutput）
gp = LocalGP_MultiOutput(2,1,200,0.1,theta_true(1),theta_true(2));
initX = [0 0;1 1;-1 0;0 2;2 2]';
for ii = 1:size(initX,2)
    gp.addPoint(initX(:,ii), d_true(initX(:,ii)));
end

%% 3) theta 估计器参数
lambda = 1;  xi = 1;  Gamma = eye(2);
P_f = zeros(1,3); b_f = 0; Qmat = zeros(2); r_vec = zeros(2,1); delta_hat = zeros(2,1);

%% 4) 仿真设置
tspan = 0:0.01:50;  dt = tspan(2)-tspan(1);
x = [2;1];   % 初始 [q;dq]
beta_hist = zeros(size(tspan));
th1_hist  = zeros(size(tspan));
th2_hist  = zeros(size(tspan));

%% 5) 主循环
for k = 1:numel(tspan)
    q  = x(1); dq = x(2); xn = [q;dq];

    [mu,s2] = gp.predict(xn); sigma = sqrt(s2);

    % beta_hat 更新
    beta_hat = beta_hat + 2*abs(q-p0)*sigma*dt;

    % theta 在线估计
    sf_bak = gp.SigmaF; ell_bak = gp.SigmaL;
    gp.SigmaF = theta_n(1); gp.SigmaL = theta_n(2);
    mu_n = gp.predict(xn);                % 名义 μ_n
    gp.SigmaF = sf_bak; gp.SigmaL = ell_bak;

    eps = 1e-6;
    gp_tmp = gp; gp_tmp.SigmaF = gp.SigmaF+eps;
    dmu_dsf = (gp_tmp.predict(xn) - mu)/eps;
    gp_tmp = gp; gp_tmp.SigmaL = gp.SigmaL+eps;
    dmu_dell = (gp_tmp.predict(xn) - mu)/eps;
    P_mu = [dmu_dsf , dmu_dell];

    f_x = (mu - Cfun(dq).*dq - gfun(q))/M;

    P_f = P_f + (-P_f + [P_mu , 1])/lambda*dt;
    b_f = b_f + (-b_f + f_x + dq)/lambda*dt;
    Pd  = P_f(1:2);
    Qmat = Qmat + (-xi*Qmat + Pd'*Pd)*dt;
    r_vec = r_vec + (-xi*r_vec + Pd'*(dq - b_f))*dt;
    delta_hat = delta_hat + (-Gamma*(Qmat*delta_hat - r_vec))*dt;

    % 控制 + 系统
    u = -Kp*(q-p0) - Kd*dq;
    ddq = (-Cfun(dq).*dq - gfun(q) + d_true(xn) + u)/M;
    x = x + [dq; ddq]*dt;

    % 记录
    beta_hist(k) = beta_hat;
    th1_hist(k)  = theta_n(1)+delta_hat(1);
    th2_hist(k)  = theta_n(2)+delta_hat(2);

    % GP 采样
    if mod(k,2)==0
        if gp.DataQuantity>=gp.MaxDataQuantity, gp.downdateParam(1); end
        gp.addPoint(xn, d_true(xn));
    end
end

%% 6) 绘图（改成 4×1 subplot）
figure('Name','Estimates','Position',[100 100 900 600]);

% ── ① β̂ 曲线 ────────────────────────────
subplot(4,1,1);
plot(tspan,beta_hist,'LineWidth',1.4);  hold on;
yline(beta_true,'--k');
grid on; ylabel('\beta_{hat}');
title('\beta_{hat}(t)');

% ── ② σ_f 估计 ───────────────────────────
subplot(4,1,2);
plot(tspan,th1_hist,'b','LineWidth',1.4);  hold on;
yline(theta_true(1),'--k');
grid on; ylabel('\sigma_f estimate');

% ── ③ ℓ  估计 ────────────────────────────
subplot(4,1,3);
plot(tspan,th2_hist,'r','LineWidth',1.4);  hold on;
yline(theta_true(2),'--k');
grid on; xlabel('Time (s)'); ylabel('l estimate');

% ── ④ 说明文本 ───────────────────────────
subplot(4,1,4);  axis off;   % 关闭坐标轴
beta_final  = beta_hist(end);
sigma_final = th1_hist(end);
ell_final   = th2_hist(end);
txt = sprintf(['Use the following value as estimation：\n' ...
               '  beta      = %.4f\n' ...
               '  sigma_f  = %.4f\n' ...
               '  l            = %.4f'], ...
               beta_final, sigma_final, ell_final);
text(0.02,0.5,txt,'FontSize',11,'FontWeight','bold','VerticalAlignment','middle');

disp('=== Simulation finished ===');
end