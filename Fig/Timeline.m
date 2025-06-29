clc; clear; close all;

%% 设置图形区域
figure;
hold on;
axis([0 3 -1 1]);  % 横轴 0~3，纵轴用于放置文本说明
axis off;          % 隐藏坐标轴

%% 绘制时间轴（水平直线）
% 从 t=0 到 t_end = 2.7（后续在 t_end 处加上箭头三角形）
plot([0, 2.7], [0, 0], 'k-', 'LineWidth', 2);

%% 绘制关键点
% t = 0
plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(0, -0.2, 't = 0 s', 'FontSize', 10, 'HorizontalAlignment', 'center');

% T_hyper，取横坐标为 1
plot(1, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(1, -0.2, 'T_{hyper}', 'FontSize', 10, 'Interpreter', 'tex', 'HorizontalAlignment', 'center');

% T_safe，取横坐标为 2
plot(2, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(2, -0.2, 'T_{safe}', 'FontSize', 10, 'Interpreter', 'tex', 'HorizontalAlignment', 'center');

%% 添加说明文本
% 第一个说明：t=0 到 T_hyper 之间（放在上方）
text((0+1)/2, 0.3, ['GP hyperparameter estimation error converges,' char(10) 'obtaining true hyperparameters'], ...
    'FontSize', 10, 'HorizontalAlignment', 'center');

% 第二个说明：T_hyper 到 T_safe 之间（放在下方）
text((1+2)/2, -0.5, 'System enters safe region within prescribed time', ...
    'FontSize', 10, 'HorizontalAlignment', 'center');

%% 在时间轴最右端加上一个接到线上的箭头（三角形）
% 用 patch 在 (2.7,0) 附近画一个三角形（与时间轴无缝衔接）
x_arrow = [2.7, 2.6, 2.6];  % 三角形三个顶点的 x 坐标
y_arrow = [0, 0.04, -0.04]; % 三角形三个顶点的 y 坐标
patch(x_arrow, y_arrow, 'k');  % 绘制黑色三角形

%% 在箭头右侧添加文字，标明 t->∞
text(2.8, 0, 't \rightarrow \infty', 'FontSize', 10, 'HorizontalAlignment', 'left', 'Interpreter', 'tex');
