%% RUN_MODEL  轮足机器人 MATLAB 建模 —— 主脚本（一键出图）
%
%   用法: 在 matlab/ 目录下直接运行 run_model
%
%   输出: figs/fig1..fig6.png  +  命令窗口的结论表
%
%   依赖: robot_params / inv_kinematics / leg_to_pulse / wip_dynamics /
%         sim_balance / lin_analysis
%         （Control System Toolbox 仅 lin_analysis 的 c2d 需要）

clc; close all;
p   = robot_params();
fig = fullfile(pwd, 'figs');
if ~exist(fig, 'dir'), mkdir(fig); end
rng(0, 'twister');

% =======================================================================
% 修复后的参数（本文档的结论，详见 README）
% =======================================================================
fix_ = p;
fix_.pid.vel_kp     = +0.05;    % 原 -0.05 符号反了
fix_.pid.balance_kd = -0.01;    % 原 -0.02 会激发 15Hz 极限环

fprintf('=============================================================\n');
fprintf(' 轮足机器人平衡控制 MATLAB 建模  %s\n', p.meta.date);
fprintf(' 控制周期 %.0f ms | 站立 %.0fmm/%.0fmm | L1=%.0f L2=%.0f\n', ...
        p.ctrl.Ts*1000, p.leg.stand_x, p.leg.stand_y, p.leg.L1, p.leg.L2);
fprintf('=============================================================\n\n');

%% ======================================================================
%% 1. 五个场景对比
%% ======================================================================
S = {};
S{end+1} = mk('A 原样固件(阻塞350ms)', p,    struct('leg',true,'block',true ,'noise',false,'T',20));
S{end+1} = mk('B 原样参数+舵机异步',   p,    struct('leg',true,'block',false,'noise',false,'T',20));
S{end+1} = mk('C 改PID+仍阻塞',        fix_, struct('leg',true,'block',true ,'noise',false,'T',20));
S{end+1} = mk('D 改PID+舵机异步',      fix_, struct('leg',true,'block',false,'noise',false,'T',20));
S{end+1} = mk('E D+传感器噪声',        fix_, struct('leg',true,'block',false,'noise',true ,'T',20));

R = cell(size(S));
fprintf('%-24s %-6s %-8s %-9s %-9s %-8s %s\n', ...
        '场景','跌倒','t_end','th_rms','th_max','腿触发','阻塞占比');
fprintf('%s\n', repmat('-', 1, 78));
for i = 1:numel(S)
    R{i} = sim_balance(S{i}.p, S{i}.sc);
    fprintf('%-24s %-6d %-8.2f %-9.2f %-9.2f %-8d %.0f%%\n', ...
        S{i}.name, R{i}.fallen, R{i}.t_end, R{i}.th_rms, R{i}.th_max, ...
        R{i}.n_trig, 100*R{i}.block_frac);
end

% ---------------- 图 1：俯仰角对比 ----------------
f = figure('Position',[80 80 1100 720], 'Visible','off');
tl = tiledlayout(f, 3, 2, 'TileSpacing','compact', 'Padding','compact');
title(tl, '五个场景的俯仰角响应（正 = 后倒）', 'FontWeight','bold');
for i = 1:numel(S)
    nexttile; hold on; grid on;
    plot(R{i}.t, R{i}.th_deg, 'LineWidth', 1.2);
    yline(0,  'k-', 'LineWidth',0.5);
    yline(10, 'r:', 'LineWidth',0.8); yline(-10,'r:','LineWidth',0.8);
    if R{i}.fallen
        plot(R{i}.t(end), R{i}.th_deg(end), 'rx', 'MarkerSize',12,'LineWidth',2);
        subtitle(sprintf('跌倒 @ %.2f s', R{i}.t_end), 'Color','r');
    end
    title(S{i}.name, 'Interpreter','none');
    xlabel('t (s)'); ylabel('\theta (deg)'); ylim([-50 50]);
end
savefig_png(f, fullfile(fig,'fig1_scenarios.png'));

%% ======================================================================
%% 2. 基线细节：B vs D
%% ======================================================================
f = figure('Position',[80 80 1150 780], 'Visible','off');
tl = tiledlayout(f, 4, 2, 'TileSpacing','compact','Padding','compact');
title(tl, '原样参数(B) 与 修正参数(D) 的逐路信号对比', 'FontWeight','bold');

rows = {
    @(o) o.th_deg,        'pitch \theta (deg)';
    @(o) o.v,             'wheel speed (m/s)';
    @(o) o.u,             'torque cmd u [-]';
    @(o) o.leg_off,       'leg offset (mm)';
    };
for r = 1:4
    nexttile; hold on; grid on;
    plot(R{2}.t, rows{r,1}(R{2}), 'LineWidth',1.1);
    title(['B  ' rows{r,2}], 'Interpreter','none'); xlabel('t (s)');
    nexttile; hold on; grid on;
    plot(R{4}.t, rows{r,1}(R{4}), 'LineWidth',1.1, 'Color',[0.85 0.33 0.1]);
    title(['D  ' rows{r,2}], 'Interpreter','none'); xlabel('t (s)');
end
savefig_png(f, fullfile(fig,'fig2_baseline.png'));

%% ======================================================================
%% 3. 高频极限环：kd = -0.02 vs -0.01
%% ======================================================================
q = p; q.pid.vel_kp = +0.05; q.pid.balance_kd = -0.02;
sc = struct('T',20,'noise',false,'leg',false,'block',false);
oa = sim_balance(q, sc);
q2 = q; q2.pid.balance_kd = -0.01;
ob = sim_balance(q2, sc);

f = figure('Position',[80 80 1150 760], 'Visible','off');
tl = tiledlayout(f, 2, 2, 'TileSpacing','compact','Padding','compact');
title(tl, 'balance\_kd 幅值对高频抖动的影响（tau\_max = 0.15 N·m）', 'FontWeight','bold');

nexttile; hold on; grid on;
plot(oa.t, oa.th_deg, 'LineWidth',0.9);
title('kd = -0.02 : 俯仰角 (deg)'); xlabel('t (s)'); ylim([-3 3]);

nexttile; hold on; grid on;
plot(ob.t, ob.th_deg, 'LineWidth',0.9, 'Color',[0.47 0.67 0.19]);
title('kd = -0.01 : 俯仰角 (deg)'); xlabel('t (s)'); ylim([-3 3]);

nexttile; hold on; grid on;
[s1,f1] = spec(oa.th_deg, oa.t, 5);
[s2,f2] = spec(ob.th_deg, ob.t, 5);
plot(f1, s1, 'LineWidth',1.2);
plot(f2, s2, 'LineWidth',1.2, 'Color',[0.47 0.67 0.19]);
xlim([0 50]); legend({'kd=-0.02','kd=-0.01'}, 'Location','northeast');
title('频谱'); xlabel('Hz'); ylabel('|X|');

nexttile; hold on; grid on;
plot(oa.t, oa.u, 'LineWidth',0.9);
yline(1,'r--'); yline(-1,'r--');
ylim([-1.4 1.4]);
title(sprintf('kd=-0.02 力矩指令（饱和占比 %.0f%%）', 100*mean(abs(oa.u(oa.t>5))>0.99)));
xlabel('t (s)'); ylabel('u [-]');

savefig_png(f, fullfile(fig,'fig3_oscillation.png'));

%% ======================================================================
%% 4. 稳定域：tau_max × kd（线性离散谱半径）
%% ======================================================================
taus = linspace(0.01, 0.40, 40);
kds  = linspace(-0.030, 0.000, 31);
SR = zeros(numel(taus), numel(kds));
for i = 1:numel(taus)
    for j = 1:numel(kds)
        q = p; q.pid.vel_kp = +0.05; q.pid.balance_kd = kds(j);
        SR(i,j) = ang_radius(lin_analysis(q, taus(i)));
    end
end

f = figure('Position',[80 80 900 620], 'Visible','off');
imagesc(kds, taus, SR); set(gca,'YDir','normal');
hold on; contour(kds, taus, SR, [1 1], 'k-', 'LineWidth',1.8);
colormap(turbo); cb = colorbar; cb.Label.String = '角度模态谱半径 max|z|';
xlabel('balance\_kd (deg/s 反馈增益)'); ylabel('tau\_max (N·m/轮)');
title({'稳定域（黑线以内 = 稳定）','注意 kd 越负、电机越强 → 越容易越过采样极限'});
plot(-0.02, 0.15, 'wp', 'MarkerSize',13, 'MarkerFaceColor','w');
text(-0.0195, 0.152, '  当前代码', 'Color','w','FontWeight','bold');
savefig_png(f, fullfile(fig,'fig4_stability_map.png'));

%% ======================================================================
%% 5. 阻塞时长扫描（固定修正参数）
%% ======================================================================
blocks = [0 20 40 60 80 100 150 200 250 300 350] / 1000;
tb = zeros(size(blocks)); fallb = zeros(size(blocks));
for i = 1:numel(blocks)
    q = fix_; q.leg.block_s = blocks(i);
    sc = struct('T',20,'noise',false,'leg',true,'block',blocks(i)>0);
    o = sim_balance(q, sc);
    tb(i) = o.t_end; fallb(i) = o.fallen;
end

f = figure('Position',[80 80 900 480], 'Visible','off');
hold on; grid on;
bar(blocks*1000, tb, 'FaceColor',[0.30 0.60 0.85]);
plot(blocks(fallb>0)*1000, tb(fallb>0), 'rx', 'MarkerSize',12, 'LineWidth',2);
xlabel('单次舵机调用阻塞时长 (ms)'); ylabel('撑住的时间 (s)');
title('腿部阻塞时长 vs 存活时间（修正 PID 后）');
legend({'存活时间','跌倒'}, 'Location','southwest');
savefig_png(f, fullfile(fig,'fig5_block_sweep.png'));

%% ======================================================================
%% 6. 逆解工作空间 + 舵机脉冲
%% ======================================================================
[X, Y] = meshgrid(0:2:100, 70:2:180);
ok = false(size(X));
AL = nan(size(X)); BE = nan(size(X));
for i = 1:numel(X)
    J = inv_kinematics(X(i), Y(i), X(i), Y(i), p);
    ok(i) = J.ok;
    if J.ok
        AL(i) = J.alpha_left*180/pi;  BE(i) = J.beta_left*180/pi;
    end
end

f = figure('Position',[80 80 1150 500], 'Visible','off');
tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
title(tl, '腿部运动学：逆解可达域与舵机脉冲', 'FontWeight','bold');

nexttile; hold on; grid on;
contourf(X, Y, double(ok), 'LineStyle','none');
plot(p.leg.stand_x, p.leg.stand_y, 'rp', 'MarkerSize',16, 'MarkerFaceColor','r');
text(p.leg.stand_x+3, p.leg.stand_y+5, '站立点 (40,90)', 'Color','r','FontWeight','bold');
xlabel('x 前倾 (mm)'); ylabel('y 高度 (mm)');
title('逆解可达域 (灰=有解)'); colormap(gca, [0.9 0.9 0.9; 0.6 0.85 0.7]);

nexttile; hold on; grid on;
xs = 0:2:100;
pl = zeros(numel(xs),4);
for i = 1:numel(xs)
    J = inv_kinematics(xs(i), p.leg.stand_y, xs(i), p.leg.stand_y, p);
    if J.ok
        pl(i,1:2) = leg_to_pulse(J.alpha_left,  J.beta_left,  'L', p);
        pl(i,3:4) = leg_to_pulse(J.alpha_right, J.beta_right, 'R', p);
    else
        pl(i,:) = NaN;
    end
end
plot(xs, pl, 'LineWidth',1.4);
legend({'L 大腿(ID2)','L 小腿(ID4)','R 大腿(ID1)','R 小腿(ID3)'}, 'Location','best');
ylabel('脉冲 (\mus)'); xlabel('x 前倾 (mm)');
title(sprintf('站立高度 y=%.0fmm 下的脉冲', p.leg.stand_y));
savefig_png(f, fullfile(fig,'fig6_kinematics.png'));

%% ======================================================================
fprintf('\n==================== 结论摘要 ====================\n');
fprintf('1. 速度环 vel_kp = %.2f 符号反了，应为正号 —— 仿真中轮速冲到 %.1f m/s、\n', ...
        p.pid.vel_kp, max(R{2}.v));
fprintf('   位移漂 %.1f m，是"越跑越快"的直接原因。\n', R{2}.x(end-1));
ph = peak_hz(oa);
if ph > 5
    fprintf('2. balance_kd = %.2f 激发 %.1f Hz 极限环，力矩 %.0f%% 时间饱和；\n', ...
            p.pid.balance_kd, ph, 100*mean(abs(oa.u(oa.t>5))>0.99));
    fprintf('   降到 -0.01 后该模态消失（幅值 %.2f deg）。\n', max(abs(ob.th_deg(ob.t>5))));
else
    fprintf('2. 真实质量(%.2f kg)下，balance_kd=%.2f 已不激发高频极限环\n', ...
            p.mech.m_total, p.pid.balance_kd);
    fprintf('   （主导 %.1f Hz 是位置漂移，不是振荡）。\n', ph);
    fprintf('   但稳定矩阵显示 kd=-0.02 只在 tau_max<=0.15 稳定，0.20 就失稳；\n');
    fprintf('   kd=-0.01 可撑到 tau_max=0.30，故仍保留 kd=-0.01 作为鲁棒选择。\n');
end
fprintf('3. 腿部阻塞 350ms：第一个触发点就跌倒（t=%.2f s）。\n', R{1}.t_end);
J0  = inv_kinematics(p.leg.stand_x, p.leg.stand_y, p.leg.stand_x, p.leg.stand_y, p);
pl0 = leg_to_pulse(J0.alpha_left,  J0.beta_left,  'L', p);
pr0 = leg_to_pulse(J0.alpha_right, J0.beta_right, 'R', p);
fprintf('4. 站立点(%.0f,%.0f)逆解: alpha=%.2f deg, beta=%.2f deg\n', ...
        p.leg.stand_x, p.leg.stand_y, J0.alpha_left*180/pi, J0.beta_left*180/pi);
fprintf('   舵机脉冲 L=[%d %d]  R=[%d %d]\n', pl0(1), pl0(2), pr0(1), pr0(2));
fprintf('5. 全部图已保存到 %s\n', fig);
fprintf('==================================================\n');

%% ====================== 局部函数 ==========================
function s = mk(name, pp, ov)
ov.name = name;
s.name = name;  s.p = pp;  s.sc = ov;
end

function savefig_png(f, path)
exportgraphics(f, path, 'Resolution', 130);
close(f);
end

function [S, f] = spec(x, t, t0)
x = x(t >= t0);  x = x(:) - mean(x);
fs = 1/median(diff(t));  n = numel(x);
Y = abs(fft(x .* hann(n))) * 2 / n;
f = (0:n-1)' * fs / n;
m = f <= fs/2;
S = Y(m);  f = f(m);
end

function hz = peak_hz(o)
[S, f] = spec(o.th_deg, o.t, 5);
S(1) = 0;
[~, i] = max(S);
hz = f(i);
end

function r = ang_radius(R)
% 剔除 z=1 处的"位置积分"结构模态（无位置反馈时必然存在）
z = R.z;
z = z(abs(z - 1) > 1e-3);
if isempty(z), r = 1; else, r = max(abs(z)); end
end
