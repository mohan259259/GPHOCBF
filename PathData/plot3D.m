% ---------------------------------------------------------------
% 参数
r  = 0.05;
p0 = [0.55, 0, 0.35];

files    = {'IdealP1.mat','NEWP1.mat'};   % 两个 .mat 文件
varNames = {'p1_traj'   ,'p1_traj'};      % 对应每个文件里的变量名

% 注意这里把 labels 定义成 1×2 的行单元格数组
labels = { ...
    'Ideal\_Trajectory', ...
    'Disturbed\_(no\_compensation)' ...
};
% ---------------------------------------------------------------
% 读取每个文件里的轨迹数据
traj = cell(numel(files),1);
for k = 1:numel(files)
    S        = load(files{k});
    traj{k}  = S.(varNames{k});   % 取出 p1_traj （N×3 矩阵）
end

% ---------------------------------------------------------------
% 3D 绘图
figure('Position',[100,80,1200,900]);   % 放大窗口
hold on;  grid on;  axis equal;
view(182, 12.5593);

colors = lines(numel(traj));
hTraj  = gobjects(numel(traj),1);

for k = 1:numel(traj)
    X = traj{k}(:,1);
    Y = traj{k}(:,2);
    Z = traj{k}(:,3);
    hTraj(k) = plot3(X, Y, Z, ...
                     'LineWidth', 3, ...
                     'Color', colors(k,:));
end

% 障碍球（浅蓝色）
[XS, YS, ZS] = sphere(40);
hSurf = surf( r*XS + p0(1), ...
              r*YS + p0(2), ...
              r*ZS + p0(3), ...
              'FaceAlpha', 0.25, ...     % 半透明
              'EdgeColor','none', ...
              'FaceColor', [0.4 0.7 1] );

% 轴标签与标题
xlabel('X (m)', 'FontSize', 14);
ylabel('Y (m)', 'FontSize', 14);
zlabel('Z (m)', 'FontSize', 14);
title('End-effector 3D Trajectories', 'FontSize', 16);

% 图例：先把所有 Handle 串成列向量，再把 labels 和新标签横向拼接
allHandles = [hTraj; hSurf];
allLabels  = [labels, {'Safety\_Zone'}];  % labels 是 1×2，{'Safety_Zone'} 是 1×1 → 1×3

% 这里将 FontSize 设置为 16，让图例框和文字都更大
legend(allHandles, allLabels, ...
       'Interpreter', 'latex', ...
       'FontSize', 16, ...
       'Location', 'best');

% ---------------------------------------------------------------
% 导出为高分辨率 PNG（尺寸扩大 2 倍）
set(gcf, 'PaperPositionMode', 'auto');
print(gcf, 'trajectory_3D_plot.png', '-dpng', '-r600');
