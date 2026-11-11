function h = draw_fivebar(ax, p, legx, legy, J, col, label, ls)
%DRAW_FIVEBAR  在 ax 上画出一套并联五连杆（腿坐标系, y 向下, 单位 mm）
%
%   J 为 inv_kinematics 的返回值（要用 alpha_left / beta_left）
%   label=true 时标注各杆长度与节点名
%
%   返回彩色连杆 (A→B) 的句柄，便于做图例

if nargin < 7, label = false; end
if nargin < 8, ls = '-'; end

% L2/L3 由 A→B、C→B 直接体现，无需显式使用
L1=p.leg.L1; L4=p.leg.L4; L5=p.leg.L5;

O  = [0 0];
O2 = [L5 0];
A  = O  + L1*[cos(J.alpha_left), sin(J.alpha_left)];
C  = O2 + L4*[cos(J.beta_left), sin(J.beta_left)];
B  = [legx, legy];

lw = 2.6;  if label, lw = 3.4; end
COL_CRANK = [0.10 0.10 0.15];
COL_FRAME = [0.45 0.45 0.50];

% ---- 机架 O—O' ----
plot(ax, [O(1) O2(1)], [O(2) O2(2)], ls, 'Color', COL_FRAME, 'LineWidth', 7);

% ---- 曲柄 O→A、O'→C ----
plot(ax, [O(1) A(1)],   [O(2) A(2)],   ls, 'Color', COL_CRANK, 'LineWidth', lw+1.4);
plot(ax, [O2(1) C(1)],  [O2(2) C(2)],  ls, 'Color', COL_CRANK, 'LineWidth', lw+1.4);

% ---- 连杆 A→B、C→B ----
plot(ax, [A(1) B(1)], [A(2) B(2)], ls, 'Color', col, 'LineWidth', lw);
h = plot(ax, [C(1) B(1)], [C(2) B(2)], ls, 'Color', col, 'LineWidth', lw);

% ---- 节点 ----
plot(ax, O(1),  O(2),  'o', 'MarkerSize',10, 'MarkerFaceColor',[0.95 0.65 0.15], 'MarkerEdgeColor','k');
plot(ax, O2(1), O2(2), 'o', 'MarkerSize',10, 'MarkerFaceColor',[0.95 0.65 0.15], 'MarkerEdgeColor','k');
plot(ax, A(1),  A(2),  'o', 'MarkerSize',6,  'MarkerFaceColor',COL_CRANK, 'MarkerEdgeColor','k');
plot(ax, C(1),  C(2),  'o', 'MarkerSize',6,  'MarkerFaceColor',COL_CRANK, 'MarkerEdgeColor','k');
plot(ax, B(1),  B(2),  'o', 'MarkerSize',8,  'MarkerFaceColor',col, 'MarkerEdgeColor','k');

if label
    text(ax, O(1)-16,  O(2)-8,  'O',  'FontSize',12,'FontWeight','bold');
    text(ax, O2(1)+8,  O2(2)-8, 'O''', 'FontSize',12,'FontWeight','bold');
    text(ax, A(1)-12,   A(2)+10, 'A',  'FontSize',12,'FontWeight','bold');
    text(ax, C(1)+6,    C(2)+10, 'C',  'FontSize',12,'FontWeight','bold');
    text(ax, B(1)+8,    B(2)+8,  'B',  'FontSize',12,'FontWeight','bold');
    % 杆长标注
    text(ax, (O(1)+A(1))/2-24, (O(2)+A(2))/2+2, 'L1=70',  'FontSize',9.5,'Color',[0.25 0.25 0.30]);
    text(ax, (O2(1)+C(1))/2-4, (O2(2)+C(2))/2-8, 'L4=70','FontSize',9.5,'Color',[0.25 0.25 0.30]);
    text(ax, (A(1)+B(1))/2-10, (A(2)+B(2))/2-4, 'L2=120','FontSize',9.5,'Color',[0.55 0.10 0.10]);
    text(ax, (C(1)+B(1))/2+4,  (C(2)+B(2))/2-4, 'L3=120','FontSize',9.5,'Color',[0.55 0.10 0.10]);
    text(ax, 26, -10, 'L5=62', 'FontSize',9.5,'Color',[0.35 0.35 0.40]);
end
end
