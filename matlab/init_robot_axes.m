function art = init_robot_axes(ax, p, ttl, col)
%INIT_ROBOT_AXES  机器人动画面板的静态图元，返回句柄供逐帧更新
%
%   腿部为**并联五连杆**（parallel 5-bar）：
%     机架 O—O' (L5) 上装两个舵机，曲柄 O→A (L1)、O'→C (L4)，
%     连杆 A→B (L2)、C→B (L3) 共同驱动末端 B（轮轴）。
%
%   art = init_robot_axes(ax, p, titleStr, color)

R = p.mech.R;
hold(ax, 'on');

% ---- 地面 ----
art.ground = plot(ax, [-10 10], [0 0], 'Color', [0.55 0.55 0.60], 'LineWidth', 2.5);

% ---- 车轮 ----
tc = linspace(0, 2*pi, 49);
art.wheel = patch(ax, 'XData', R*cos(tc), 'YData', R*sin(tc), ...
    'FaceColor', [0.22 0.22 0.26], 'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 1.2);

% ---- 轮辐（显示转动）----
art.sp1 = plot(ax, [0 0], [0 0], '-', 'Color', [0.95 0.65 0.15], 'LineWidth', 2);
art.sp2 = plot(ax, [0 0], [0 0], '-', 'Color', [0.95 0.65 0.15], 'LineWidth', 2);

% ---- 五连杆：机架 / 曲柄 / 连杆 ----
art.frame  = plot(ax, [0 0], [0 0], '-', 'Color', [0.45 0.45 0.50], 'LineWidth', 5);   % O—O'
art.crank1 = plot(ax, [0 0], [0 0], '-', 'Color', [0.20 0.20 0.25], 'LineWidth', 4);   % O→A
art.crank2 = plot(ax, [0 0], [0 0], '-', 'Color', [0.20 0.20 0.25], 'LineWidth', 4);   % O'→C
art.link1  = plot(ax, [0 0], [0 0], '-', 'Color', col, 'LineWidth', 2.5);              % A→B
art.link2  = plot(ax, [0 0], [0 0], '-', 'Color', col, 'LineWidth', 2.5);              % C→B
art.pivot1 = plot(ax, [0 0], [0 0], 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.95 0.65 0.15], 'MarkerEdgeColor', [0.3 0.3 0.3]);
art.pivot2 = plot(ax, [0 0], [0 0], 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.95 0.65 0.15], 'MarkerEdgeColor', [0.3 0.3 0.3]);

% ---- 机身（装在机架上，随 θ 旋转）----
art.body = patch(ax, 'XData', zeros(1,4), 'YData', zeros(1,4), ...
    'FaceColor', [0.93 0.94 0.97], 'EdgeColor', col, 'LineWidth', 2);

% ---- 顶部传感器 ----
art.sensor = patch(ax, 'XData', zeros(1,4), 'YData', zeros(1,4), ...
    'FaceColor', col, 'EdgeColor', 'none');

% ---- 文字 ----
art.title  = title(ax, ttl, 'FontSize', 11, 'Interpreter', 'none');
art.status = text(ax, 0, 0.405, '', 'FontSize', 14, 'FontWeight', 'bold', ...
    'Color', col, 'HorizontalAlignment', 'center', 'Interpreter', 'none');
art.info   = text(ax, 0, 0.368, '', 'FontSize', 9.5, 'Color', [0.30 0.30 0.35], ...
    'HorizontalAlignment', 'center', 'Interpreter', 'tex');

set(ax, 'XLim', [-0.30 0.30], 'YLim', [-0.03 0.43]);
% 注意: 不能用 Visible='off'，那会把 title 一起隐藏
set(ax, 'Visible','on', 'XTick',[], 'YTick',[], 'Box','off', 'Color','none');
set(ax.XAxis, 'Visible','off');
set(ax.YAxis, 'Visible','off');
end
