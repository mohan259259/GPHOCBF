clc; clear; close all;

% 图形设置
figure; hold on; axis equal;
xlim([-1.5, 1.5]); ylim([-1.5, 1.5]);
set(gca, 'Color', 'w');

% ==== unsafe zone 背景（红色） ====
fill([-1.5, 1.5, 1.5, -1.5], [-1.5, -1.5, 1.5, 1.5], ...
    [1, 0.8, 0.8], 'EdgeColor', 'none');
rectangle('Position', [-1.5, -1.5, 3, 3], ...
    'EdgeColor', 'k', 'LineWidth', 2); % 黑色边框
text(0.4, -1.4, 'Unsafe zone', 'FontSize', 15, 'FontAngle', 'italic');

% ==== safe zone 圆 ====
r = 0.7;
center = [0.5, -0.5];
theta = linspace(0, 2*pi, 200);
x_circle = r * cos(theta) + center(1);
y_circle = r * sin(theta) + center(2);
fill(x_circle, y_circle, [0.7 0.9 1], ...
    'EdgeColor', 'k', 'LineWidth', 1.5);
text(center(1)-0.25, center(2), 'Safe zone', ...
    'FontSize', 11, 'FontWeight', 'bold');

% ==== 贝塞尔曲线轨迹 ====
p0 = [-1, 1];        % 初始点
p1 = [-0.6, 0.6];    % 控制点1
p2 = [0.1, 0.06];    % 控制点2
p3 = [0.5, -0.2];    % 终点（用于轨迹计算，但不绘制）

t = linspace(0, 1, 100);
B = (1 - t).^3 .* p0' + ...
    3 * (1 - t).^2 .* t .* p1' + ...
    3 * (1 - t) .* t.^2 .* p2' + ...
    t.^3 .* p3';
x_traj = B(1, :);
y_traj = B(2, :);
plot(x_traj, y_traj, 'k-', 'LineWidth', 1.5);

% ==== 初始点 ====
plot(p0(1), p0(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
text(p0(1) - 0.3, p0(2) + 0.1, 'Initial point (unsafe)', 'FontSize', 15);

% ==== T_hyper ====
T_idx = 35;
plot(x_traj(T_idx), y_traj(T_idx), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
text(x_traj(T_idx)-0.35, y_traj(T_idx)-0.15, 'T_{hyper}', ...
    'FontSize', 12, 'Interpreter', 'tex');

% ==== T_safe （指定坐标改为 [0.1, 0.06]）====
T_safe_target = [0.1, 0.06];
dist = sqrt((x_traj - T_safe_target(1)).^2 + (y_traj - T_safe_target(2)).^2);
[~, T_safe_idx] = min(dist);
plot(x_traj(T_safe_idx), y_traj(T_safe_idx), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
text(x_traj(T_safe_idx)+0.05, y_traj(T_safe_idx)+0.05, 'T_{safe}', ...
    'FontSize', 12, 'Interpreter', 'tex');

% ==== （原 p3 终点绘制已移除）====
% plot(p3(1), p3(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);

% ==== 深红色箭头已移除 ====
% （相关代码已全部注释，不再绘制方向箭头）

% ==== 去除坐标轴 ====
axis off;
