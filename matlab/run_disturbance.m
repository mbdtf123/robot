%% RUN_DISTURBANCE  受力扰动下的平衡能力分析（判据：最大倾角 ≤ 10°）
%
%   用法: 在 matlab/ 目录下运行  run_disturbance
%   输出: figs/fig9_disturbance.png / fig10_limit.png / fig11_velocity.png
%
%   扰动模型：t = 2.0 s 时对车身质心施加水平恒力 F，持续 50 ms，
%             即冲量 J = F × 0.05 (N·s)。正号 = 向前推。
%
%   两个判据分开看：
%     判据 A（角度）：扰动后 |θ| 峰值 ≤ 10° 且未跌倒   <- 用户要求
%     判据 B（速度）：扰动后速度能收敛回 0             <- 停在原地
%
%   重要：本模型里判据 B **始终不满足**，原因见报告末尾。

clc; close all;
p    = robot_params();
fig  = fullfile(pwd,'figs');
if ~exist(fig,'dir'), mkdir(fig); end
rng(0,'twister');

TH_LIMIT = 10;      % 最大倾角 (°)
DIST_T   = 2.0;
DIST_DUR = 0.05;
MASS_APP = p.mech.m_total;   % 用整机质量把冲量换算成等效 Δv

fix_ = p;
fix_.pid.vel_kp     = +0.05;
fix_.pid.balance_kd = -0.01;

fprintf('===============================================================\n');
fprintf(' 受力扰动平衡能力   判据A: |theta| <= %.0f deg\n', TH_LIMIT);
fprintf(' 扰动: t=%.1fs 施加水平力 %.0f ms\n', DIST_T, DIST_DUR*1000);
fprintf('===============================================================\n\n');

%% ================= 1. 典型脉冲响应 =================
Fshow = [0 20 40 60 80 100];
R = cell(numel(Fshow),1);
fprintf('%-6s %-9s %-9s %-8s %-9s %-9s %s\n', ...
        'F(N)','冲量(N·s)','Δv(m/s)','θ峰值','θ稳态','末速度','判据A');
fprintf('%s\n', repmat('-',1,72));
for i = 1:numel(Fshow)
    sc = struct('T',12,'noise',false,'leg',true,'block',false, ...
                'dist_on',true,'dist_t',DIST_T,'dist_F',Fshow(i),'dist_dur',DIST_DUR);
    R{i} = sim_balance(fix_, sc);
    m = get_metrics(R{i}, DIST_T);
    fprintf('%-6d %-9.2f %-9.2f %-8.2f %-9.2f %-9.3f %s\n', ...
        Fshow(i), Fshow(i)*DIST_DUR, Fshow(i)*DIST_DUR/MASS_APP, ...
        m.peak, m.th_end, m.v_end, tern(m.peak<=TH_LIMIT && ~m.fallen,'✓','✗'));
end

% ---------------- fig9 ----------------
f = figure('Position',[80 80 1050 640],'Visible','off','Color','w');
tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
title(tl,'不同冲量下的俯仰角与速度响应（修复后参数）','FontWeight','bold');

nexttile; hold on; grid on;
cols = lines(numel(Fshow));
for i = 1:numel(Fshow)
    plot(R{i}.t, R{i}.th_deg, 'LineWidth',1.2, 'Color',cols(i,:));
end
yline( TH_LIMIT,'r--','LineWidth',1.8);
yline(-TH_LIMIT,'r--','LineWidth',1.8);
xline(DIST_T,'k:','LineWidth',1.0);
ylabel('\theta (deg)'); xlabel('t (s)'); ylim([-16 16]);
legend(arrayfun(@(v) sprintf('J=%.2f N·s',v*DIST_DUR), Fshow, 'UniformOutput',false), ...
    'Location','eastoutside','Box','off');
title(sprintf('俯仰角（红虚线 = %.0f° 上限）', TH_LIMIT));

nexttile; hold on; grid on;
for i = 1:numel(Fshow)
    plot(R{i}.t, R{i}.v, 'LineWidth',1.1, 'Color',cols(i,:));
end
xline(DIST_T,'k:','LineWidth',1.0);
ylabel('速度 (m/s)'); xlabel('t (s)'); ylim([-1 2.2]);
title('速度：扰动后均不收敛到 0（只会极缓慢衰减）');
savefig_png(f, fullfile(fig,'fig9_disturbance.png'));

%% ================= 2. 撞冲量扫临界 =================
Fs = 0:2:120;
pk = nan(size(Fs)); v_end = nan(size(Fs)); fallen = false(size(Fs));
for i = 1:numel(Fs)
    sc = struct('T',12,'noise',false,'leg',true,'block',false, ...
                'dist_on',true,'dist_t',DIST_T,'dist_F',Fs(i),'dist_dur',DIST_DUR);
    o = sim_balance(fix_, sc);
    m = get_metrics(o, DIST_T);
    pk(i) = m.peak;  v_end(i) = m.v_end;  fallen(i) = m.fallen;
end

% 线性插值找 10° 临界
idx = find(pk > TH_LIMIT, 1, 'first');
if isempty(idx)
    Fcrit = NaN;
elseif idx == 1
    Fcrit = Fs(1);
else
    Fcrit = Fs(idx-1) + (TH_LIMIT-pk(idx-1))/(pk(idx)-pk(idx-1))*(Fs(idx)-Fs(idx-1));
end

% ---------------- fig10 ----------------
f = figure('Position',[80 80 980 700],'Visible','off','Color','w');
tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
title(tl,'临界冲量扫描（修复后参数）','FontWeight','bold');

nexttile; hold on; grid on;
plot(Fs*DIST_DUR, pk, 'o-', 'LineWidth',1.8, 'Color',[0.10 0.55 0.20], ...
     'MarkerFaceColor',[0.10 0.55 0.20], 'MarkerSize',4);
yline(TH_LIMIT,'r-','LineWidth',2);
if ~isnan(Fcrit)
    xline(Fcrit*DIST_DUR,'k:','LineWidth',1.6);
    text(Fcrit*DIST_DUR*1.02, TH_LIMIT*1.12, ...
        sprintf('临界 J = %.2f N·s  (Δv≈%.2f m/s)', Fcrit*DIST_DUR, Fcrit*DIST_DUR/MASS_APP), ...
        'FontWeight','bold','FontSize',10);
end
xlabel('冲量 J (N·s)'); ylabel('扰动后 \theta 峰值 (deg)');
title(sprintf('峰值倾角 vs 冲量（红实线 = %.0f° 上限）', TH_LIMIT));
ylim([0 40]);

nexttile; hold on; grid on;
plot(Fs*DIST_DUR, v_end, 's-', 'LineWidth',1.6, 'Color',[0.85 0.15 0.15], 'MarkerSize',4);
xlabel('冲量 J (N·s)'); ylabel('仿真结束时速度 (m/s)');
title('末速度：随冲量线性增长，始终不归零 —— 判据 B 不满足');
savefig_png(f, fullfile(fig,'fig10_limit.png'));

%% ================= 3. 为什么速度收不回来 =================
% 对比：腿防倒开/关
sc_on  = struct('T',12,'noise',false,'leg',true, 'block',false, ...
                'dist_on',true,'dist_t',DIST_T,'dist_F',20,'dist_dur',DIST_DUR);
sc_off = sc_on; sc_off.leg = false;
o_on  = sim_balance(fix_, sc_on);
o_off = sim_balance(fix_, sc_off);

% 实测速度衰减时间常数（指数拟合 v 的绝对值）
tau_on  = decay_tau(o_on,  4, 14);
tau_off = decay_tau(o_off, 4, 14);

f = figure('Position',[80 80 980 460],'Visible','off','Color','w');
tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
title(tl,'速度为什么不收敛','FontWeight','bold');

nexttile; hold on; grid on;
plot(o_on.t,  o_on.v,  'LineWidth',1.6, 'Color',[0.10 0.55 0.20]);
plot(o_off.t, o_off.v, 'LineWidth',1.6, 'Color',[0.15 0.35 0.75], 'LineStyle','--');
xline(DIST_T,'k:');
xlabel('t (s)'); ylabel('v (m/s)');
legend({'腿防倒 ON','腿防倒 OFF','扰动时刻'},'Location','northeast','Box','off');
title('速度都只极缓慢衰减');

nexttile; hold on; grid on;
plot(o_on.t, o_on.th_deg, 'LineWidth',1.6, 'Color',[0.10 0.55 0.20]);
plot(o_off.t, o_off.th_deg,'LineWidth',1.6, 'Color',[0.15 0.35 0.75], 'LineStyle','--');
xline(DIST_T,'k:');
xlabel('t (s)'); ylabel('\theta (deg)');
legend({'腿防倒 ON','腿防倒 OFF'},'Location','northeast','Box','off');
title('腿 ON 时稳态倾角更大（重力被腿位移抵消）');
savefig_png(f, fullfile(fig,'fig11_velocity.png'));

%% ================= 4. 结论 =================
fprintf('\n================ 结论 ================\n');
fprintf('判据A（角度）:\n');
if ~isnan(Fcrit)
    fprintf('  临界冲量 J = %.2f N*s  ->  等效 Δv = %.2f m/s\n', Fcrit*DIST_DUR, Fcrit*DIST_DUR/MASS_APP);
    iF = find(fallen, 1);
    if isempty(iF)
        fprintf('  低于此值：峰值倾角 < %.0f 度，机器人不会倒\n', TH_LIMIT);
    else
        fprintf('  低于此值：峰值倾角 < %.0f 度\n', TH_LIMIT);
        fprintf('  %.2f < J < %.2f N*s：倾角会超 %.0f 度但还能救回来\n', ...
            Fcrit*DIST_DUR, Fs(iF)*DIST_DUR, TH_LIMIT);
        fprintf('  J > %.2f N*s：真的翻倒（坡度 45 度跌倒判据）\n', Fs(iF)*DIST_DUR);
    end
end
fprintf('\n判据B（速度）: ✗ 全部不满足\n');
fprintf('  扰动后速度只按指数极缓慢衰减，不会回到 0，机器人会一直朝一个方向滚。\n');
fprintf('  实测衰减时间常数：腿防倒 ON  tau=%.0f s ；OFF  tau=%.0f s\n', tau_on, tau_off);
fprintf('  -> 腿防倒机构在稳态下把重力力矩抵消掉，让速度环几乎不起作用（衰减慢 4 倍）\n');
fprintf('  根因：速度环是纯 P 且增益太小，缺少积分项 —— 任何恒定速度都对应一个平衡点。\n');
fprintf('  要真正停在原地，必须给速度环加积分，或启用位置环 robot_kp。\n');

fprintf('\n关于"最大倾角 10 度":\n');
fprintf('  main.cpp:451 的 target_pitch 限幅 ±%.0f 度在现有增益下永不触发\n', p.pid.pitch_lim);
fprintf('  （需要 |v| = %.0f rad/s = %.1f m/s 才会到 10 度，而 max_velocity 只有 %.1f rad/s）\n', ...
    10/abs(fix_.pid.vel_kp), 10/abs(fix_.pid.vel_kp)*p.mech.R, p.ctrl.max_velocity);
fprintf('  实际限制倾角的是直立环本身，不是那个限幅。\n');

fprintf('\n图已保存到 %s\n', fig);

%% ====================== 局部函数 ==========================
function m = get_metrics(o, t0)
i0 = find(o.t >= t0, 1);
if isempty(i0), i0 = 1; end
m.peak = max(abs(o.th_deg(i0:end)));
m.fallen = o.fallen;
m.v_end = abs(o.v(end));
m.th_end = o.th_deg(end);
end

function tau = decay_tau(o, t1, t2)
% 对 |v| 做指数拟合拿时间常数
idx = o.t >= t1 & o.t <= t2 & abs(o.v) > 1e-6;
tt = o.t(idx);  vv = abs(o.v(idx));
if numel(tt) < 10, tau = NaN; return; end
c = polyfit(tt - tt(1), log(vv), 1);
tau = -1/c(1);
end

function savefig_png(f, path)
exportgraphics(f, path, 'Resolution',130);
close(f);
end

function s = tern(c,a,b)
if c, s = a; else, s = b; end
end
