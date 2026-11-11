%% PLOT_LINKAGE  并联五连杆机构图
%
%   左：站立点构型，标注 5 根杆与 2 个舵机
%   右：防倒工作范围 leg_x = 20~60 mm 内，两个曲柄如何协同变形
%
%   输出 figs/fig7_fivebar.png

clc; close all;
p = robot_params();
fig = fullfile(pwd,'figs');
if ~exist(fig,'dir'), mkdir(fig); end

L1=p.leg.L1; L2=p.leg.L2; L3=p.leg.L3; L4=p.leg.L4; L5=p.leg.L5;

%% ---------------- 左：站立点构型 ----------------
f = figure('Position',[80 80 1200 560],'Visible','off','Color','w');
tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
title(tl,'并联五连杆（parallel 5-bar）机构','FontWeight','bold','FontSize',13);

ax1 = nexttile; hold(ax1,'on'); axis(ax1,'equal'); grid(ax1,'on');
J = inv_kinematics(40, 90, 40, 90, p);
draw_fivebar(ax1, p, 40, 90, J, [0.15 0.35 0.75], true, '-');
plot(ax1, 40, 90, 'p', 'MarkerSize',18, 'MarkerFaceColor',[0.85 0.15 0.15],'MarkerEdgeColor','k');
text(ax1, 46, 96, 'B 轮轴 (40,90)', 'FontSize',9,'Color',[0.6 0 0]);
xlim(ax1,[-95 165]); ylim(ax1,[-22 115]);
set(ax1,'YDir','reverse');
xlabel(ax1,'x 前向 (mm)'); ylabel(ax1,'y 向下 (mm)');
title(ax1,'站立点构型   \alpha=155.21°  \beta=9.69°');

%% ---------------- 右：防倒范围内协同变形 ----------------
ax2 = nexttile; hold(ax2,'on'); grid(ax2,'on');
xs = [20 30 40 50 60];
cols = [0.20 0.45 0.85; 0.30 0.65 0.95; 0.15 0.15 0.15; 0.95 0.55 0.15; 0.85 0.15 0.15];
leg_h = gobjects(numel(xs),1);
for i = 1:numel(xs)
    Jx = inv_kinematics(xs(i), 90, xs(i), 90, p);
    leg_h(i) = draw_fivebar(ax2, p, xs(i), 90, Jx, cols(i,:), false, '-');
end
legend(ax2, leg_h, arrayfun(@(v) sprintf('leg_x = %d mm', v), xs, 'UniformOutput',false), ...
    'Location','southoutside','Orientation','horizontal','Box','off');
axis(ax2,'equal');
xlim(ax2,[-95 165]); ylim(ax2,[-25 120]);
set(ax2,'YDir','reverse');
xlabel(ax2,'x 前向 (mm)'); ylabel(ax2,'y 向下 (mm)');
title(ax2,'防倒范围 leg\_x=20~60mm：两曲柄协同');

exportgraphics(f, fullfile(fig,'fig7_fivebar.png'), 'Resolution',130);
close(f);

%% ---------------- 图 8：小腿舵机脉冲在 leg_x≈53mm 处跳变 ----------------
xs2 = 20:0.5:60;
pl = nan(numel(xs2),2);  pr = nan(numel(xs2),2);
beta_deg = nan(numel(xs2),1);
for i = 1:numel(xs2)
    Jx = inv_kinematics(xs2(i), 90, xs2(i), 90, p);
    beta_deg(i) = Jx.beta_left*180/pi;
    if Jx.ok
        pl(i,:) = leg_to_pulse(Jx.alpha_left,  Jx.beta_left,  'L', p);
        pr(i,:) = leg_to_pulse(Jx.alpha_right, Jx.beta_right, 'R', p);
    end
end

f2 = figure('Position',[80 80 980 520],'Visible','off','Color','w');
tl2 = tiledlayout(f2,1,2,'TileSpacing','compact','Padding','compact');
title(tl2,'防倒偏移引起的小腿舵机脉冲跳变（并联五连杆 beta 过零）','FontWeight','bold');

nexttile; hold on; grid on;
plot(xs2, beta_deg, 'LineWidth',1.8, 'Color',[0.15 0.35 0.75]);
yline(0, 'k-'); yline(90,'r--');
ylabel('\beta (deg)'); xlabel('leg\_x (mm)');
title('\beta 角：约 52.5 mm 处跨过 0°，被归一化到 ~360°');
legend({'','\beta=0','\beta=90 (选择上限)'},'Location','best','Box','off');

nexttile; hold on; grid on;
plot(xs2, pl(:,2), 'LineWidth',1.8, 'Color',[0.85 0.15 0.15]);
plot(xs2, pr(:,2), 'LineWidth',1.4, 'Color',[0.10 0.55 0.20], 'LineStyle','--');
xline(52.5, 'k:', 'LineWidth',1.2);
ylabel('小腿舵机脉冲 (\mus)'); xlabel('leg\_x (mm)');
title('脉冲从 2033 突降到 1444（跳 589 \mus）');
legend({'左腿 ID4','右腿 ID3','跳变点'},'Location','southwest','Box','off');

exportgraphics(f2, fullfile(fig,'fig8_pulse_jump.png'), 'Resolution',130);
close(f2);

%% ---------------- 数值明细 ----------------
fprintf('leg_x  ->  alpha(deg)  beta(deg)   d_alpha/d_x (deg/mm)\n');
prev = [];
for x = [20 30 40 50 60]
    Jx = inv_kinematics(x, 90, x, 90, p);
    a = Jx.alpha_left*180/pi;  b = Jx.beta_left*180/pi;
    if isempty(prev), dd = NaN; else, dd = (a-prev(1))/(x-prev(3)); end
    fprintf('%5d  ->  %8.2f  %8.2f   %+8.2f\n', x, a, b, dd);
    prev = [a b x];
end
fprintf('图已保存: %s\n', fullfile(fig,'fig7_fivebar.png'));
