% ---------------------------------------------------------------
% 参数
r  = 0.05;
p0 = [0.55, 0, 0.35];

files    = {'IdealP1.mat','SigamP1.mat','SigmaESP1.mat'};%'DisP1.mat'};
varNames = {'p1_traj'   ,'p1_traj'   ,'p1_traj' };%'p1_traj'  };

labels = { ...
    'Ideal_Trajectory', ...
    'GP_Nominal', ...
    'GP_Estimated' ...
    %'Disturbed (no compensation)', ...
};
% ---------------------------------------------------------------
% 读取 4 条轨迹
traj = cell(numel(files),1);
for k = 1:numel(files)
    S        = load(files{k});
    traj{k}  = S.(varNames{k});
end
% ---------------------------------------------------------------
% 3D 绘图
figure('Position',[100,80,1200,900]);           %%% <<< ① 放大窗口
hold on; grid on; axis equal;
view(182,12.5593);

colors = lines(numel(traj));
hTraj  = gobjects(numel(traj),1);

for k = 1:numel(traj)
    X = traj{k}(:,1);  Y = traj{k}(:,2);  Z = traj{k}(:,3);
    hTraj(k) = plot3(X,Y,Z,'LineWidth',3,'Color',colors(k,:));
end

% 障碍球（浅蓝色）
[XS,YS,ZS] = sphere(40);
hSurf = surf(r*XS + p0(1), r*YS + p0(2), r*ZS + p0(3), ...
             'FaceAlpha',0.25, ...                       % 透明度保持
             'EdgeColor','none', ...
             'FaceColor',[0.4 0.7 1]);       %%% <<< ② 浅蓝色

% -------- 删除 nominal 黑线：不再绘制 ---------

% 轴标签与标题
xlabel('X (m)','FontSize',14);
ylabel('Y (m)','FontSize',14);
zlabel('Z (m)','FontSize',14);
title('End-effector 3D Trajectories','FontSize',16);

% 图例
allHandles = [hTraj ; hSurf];
allLabels  = [labels , {'Safety_Zone'}];
legend(allHandles, allLabels,'Interpreter','latex','Location','best');

% ---------------------------------------------------------------
% 导出为高分辨 PNG（尺寸扩大 2 倍）
set(gcf,'PaperPositionMode','auto');
print(gcf,'trajectory_3D_plot.png','-dpng','-r600');   %%% <<< ③ 分辨率 ×2
