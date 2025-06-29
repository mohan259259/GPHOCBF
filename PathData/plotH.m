%% ------------------------------------------------------------
% 文件与标签
files  = {'IdealH1.mat','DisH1.mat','GPH1.mat','EstH1.mat'};
labels = {'Ideal'      ,'Disturbance' ,'GP_ Nominal','GP_ Estimation'};

colors = lines(numel(files));

figure('Position',[100 100 900 500]);
hold on; grid on;

%% ---------------------------------------------------------------
for k = 1:numel(files)
    S = load(files{k});

    % ---------- 找到 h 向量和对应时间 ----------
    if isfield(S,'h1')
        h = S.h1;
    else
        error('文件 %s 中找不到变量 h1', files{k});
    end

    if isfield(S,'t')                        % 情形 3
        t = S.t(:);
    elseif ismatrix(h) && size(h,2) == 2    % 情形 1
        t = h(:,1);   h = h(:,2);
    else                                    % 情形 2 ：单列
        dt = 0.004;                         % ← 采样间隔 0.004 s
        t  = (0:numel(h)-1)' * dt;
    end
    % -----------------------------------------

    plot(t, h, 'LineWidth', 1.6, ...
         'Color', colors(k,:), ...
         'DisplayName', labels{k});
end

%% t = 1 s 虚线
xline(1, 'k--', 'LineWidth', 1.2, 'DisplayName', 't = 1 s');
yline(0,'k--','LineWidth',1.2,'DisplayName','h_1 = 0');
xlabel('Time  t  (s)');
ylabel('Safety function  h_1');
title('Safety Function Trajectories');
legend('Location', 'best');