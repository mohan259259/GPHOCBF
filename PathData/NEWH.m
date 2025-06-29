%% ------------------------------------------------------------
% 文件与标签
files  = {'IdealH1.mat','NEWDIS.mat','NEWGPEST.mat','NEWGPEB.mat','EstH1.mat','NEWGPEBNO.mat'};
labels = {'Ideal'      ,'Disturbance' ,'GP_ ONLY','ErrorB_ ONLY','GP_ COM_ Nom','GP_ COM_ Est'};

colors = lines(numel(files));

figure('Position',[100 100 900 500]);
hold on; grid on;

%% ---------------------------------------------------------------
for k = 1:numel(files)
    S = load(files{k});

    % ---------- 找到 h 向量 和 对应时间 ----------
    if isfield(S,'h1')
        h = S.h1;
    elseif isfield(S,'h0')
        h = S.h0;
    else
        error('文件 %s 中找不到变量 h1 或 h0', files{k});
    end

    if isfield(S,'t')                        % 情形 3：有单独的 t 变量
        t = S.t(:);
    elseif ismatrix(h) && size(h,2) == 2     % 情形 1：h 是两列，第一列是时间
        t = h(:,1);
        h = h(:,2);
    else                                     % 情形 2：h 是单列，用固定 dt 重建 t
        dt = 0.004;                          % ← 采样间隔 0.004 s
        t  = (0:numel(h)-1)' * dt;
    end
    % -----------------------------------------

    plot(t, h, 'LineWidth', 1.6, ...
         'Color', colors(k,:), ...
         'DisplayName', labels{k});
end

%% t = 1 s 虚线
xline(1, 'k--', 'LineWidth', 1.2, 'DisplayName', 't = 1 s');
yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'h = 0');
xlabel('Time  t  (s)');
ylabel('Safety function  h');
title('Safety Function Trajectories');
legend('Location', 'best');
