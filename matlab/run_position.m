%% RUN_POSITION  位置环（robot_kp）验证与整定
%
%   用法: 在 matlab/ 目录下运行  run_position
%   输出: figs/fig12_position.png / fig13_posgain.png / fig14_posfinal.png
%
%   背景
%     固件原来的位置环保存在 adjust_leg_position_for_balance() 里，把位置误差
%     加到**腿部偏移**上。但腿部偏移只改变平衡点、不产生持续力矩，实测无法把
%     机器人拉回原点。正确接法是加到**目标俯仰角**上，通过轮子力矩控位置。
%
%   另一个关键：6050 里写死的 +0.6° 俯仰偏置会变成位置环的稳态误差
%     稳态误差 = PITCH_BIAS_DEG / robot_kp
%     实测 k=1/2/3 -> 0.599/0.301/0.201 m，与 0.6/k 严格吻合。
%
%   结论：启用位置环必须同时把 bias 归零。

clc; close all;
p   = robot_params();
fig = fullfile(pwd,'figs');
if ~exist(fig,'dir'), mkdir(fig); end

% 修复后的控制参数
q = p;
q.pid.vel_kp     = +0.05;
q.pid.balance_kd = -0.01;
q.sens.pitch_bias = 0.0;      % [FIX-8] 去掉硬偏置

DIST = struct('T',100,'noise',false,'leg',true,'block',false, ...
              'dist_on',true,'dist_t',2.0,'dist_F',20,'dist_dur',0.05);

fprintf('========================================================\n');
fprintf(' 位置环验证  扰动 J = %.2f N·s\n', DIST.dist_F*DIST.dist_dur);
fprintf('========================================================\n\n');

%% ================= 1. 三种接法对比 =================
cfgs = {
    '无位置环',            false, 'pitch', 0
    '旧接法: 腿偏移 k=50', true,  'leg',   50
    '新接法: 俯仰角 k=2',  true,  'pitch', 2
};
R = cell(size(cfgs,1),1);
fprintf('%-22s %-9s %-11s %-11s %s\n','配置','峰值θ','末5s|x|','末5s|v|','判定');
for i = 1:size(cfgs,1)
    sc = DIST;
    sc.pos_on = cfgs{i,2};  sc.pos_mode = cfgs{i,3};  sc.k_pos = cfgs{i,4};
    sc.pos_target = 0;
    R{i} = sim_balance(q, sc);
    m = R{i}.t >= R{i}.t_end-5;
    xe = max(abs(R{i}.x(m)));  ve = max(abs(R{i}.v(m)));
    ok = ~R{i}.fallen && xe < 0.05 && ve < 0.02;
    fprintf('%-22s %-9.2f %-11.3f %-11.3f %s\n', cfgs{i,1}, ...
        max(abs(R{i}.th_deg(R{i}.t>=2))), xe, ve, tern(ok,'✓ 收敛','✗'));
end

% ---------------- fig12 ----------------
f = figure('Position',[80 80 1000 620],'Visible','off','Color','w');
tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
title(tl,'位置环三种接法对比（扰动 J=1.00 N·s）','FontWeight','bold');
cols = [0.85 0.15 0.15; 0.95 0.60 0.10; 0.10 0.55 0.20];

nexttile; hold on; grid on;
for i=1:3, plot(R{i}.t, R{i}.x, 'LineWidth',1.5, 'Color',cols(i,:)); end
xline(2.0,'k:','LineWidth',1.0);
xlabel('t (s)'); ylabel('位置 x (m)');
legend(cfgs(:,1),'Location','northeast','Box','off');
title('位置：无位置环会一直漂（注意 100 s 尺度的 60 m）');

nexttile; hold on; grid on;
for i=1:3, plot(R{i}.t, R{i}.v, 'LineWidth',1.4, 'Color',cols(i,:)); end
xline(2.0,'k:','LineWidth',1.0);
xlabel('t (s)'); ylabel('速度 (m/s)');
legend(cfgs(:,1),'Location','northeast','Box','off');
title('速度：只有新接法能收敛到 0');
savefig_png(f, fullfile(fig,'fig12_position.png'));

%% ================= 2. bias 的影响 =================
fprintf('\n--- pitch_bias 造成的稳态位置误差 ---\n');
ks = [1 2 3 4 6];
bias_list = [0.6 0.0];
fprintf('%-8s %-12s %-12s %-12s %s\n','k','理论 0.6/k','bias=0.6 实测','bias=0 实测','bias=0 判定');
xe_bias = nan(numel(ks),2);
for i = 1:numel(ks)
    for j = 1:2
        qq = q; qq.sens.pitch_bias = bias_list(j);
        sc = DIST; sc.pos_on = true; sc.pos_mode = 'pitch'; sc.k_pos = ks(i); sc.pos_target = 0;
        o = sim_balance(qq, sc);
        xe_bias(i,j) = abs(o.x(end));
    end
    fprintf('%-8.0f %-12.3f %-12.3f %-12.4f %s\n', ks(i), 0.6/ks(i), ...
        xe_bias(i,1), xe_bias(i,2), tern(xe_bias(i,2)<0.02,'✓ 到原点','✗'));
end

% ---------------- fig13 ----------------
f = figure('Position',[80 80 900 500],'Visible','off','Color','w');
hold on; grid on;
plot(ks, 0.6./ks, 'k--', 'LineWidth',1.5);
plot(ks, xe_bias(:,1), 'o-', 'LineWidth',1.8, 'Color',[0.85 0.15 0.15], ...
     'MarkerFaceColor',[0.85 0.15 0.15], 'MarkerSize',7);
plot(ks, xe_bias(:,2), 's-', 'LineWidth',1.8, 'Color',[0.10 0.55 0.20], ...
     'MarkerFaceColor',[0.10 0.55 0.20], 'MarkerSize',7);
set(gca,'YScale','log');
xlabel('robot\_kp (deg/m)'); ylabel('稳态位置误差 |x| (m)');
legend({'理论 0.6/k','实测 bias=+0.6°','实测 bias=0'},'Location','northeast','Box','off');
title('俯仰偏置 -> 位置稳态误差：bias 归零后误差降到 mm 级');
savefig_png(f, fullfile(fig,'fig13_posgain.png'));

%% ================= 3. 推荐配置的完整响应 =================
k_best = 2.0;
sc = DIST; sc.pos_on = true; sc.pos_mode = 'pitch'; sc.k_pos = k_best; sc.pos_target = 0;
o = sim_balance(q, sc);

f = figure('Position',[80 80 1050 720],'Visible','off','Color','w');
tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
title(tl, sprintf('推荐配置：bias=0, robot\\_kp=%.0f deg/m（扰动 J=1.00 N·s）', k_best), ...
      'FontWeight','bold');

nexttile; hold on; grid on;
plot(o.t, o.th_deg, 'LineWidth',1.5, 'Color',[0.15 0.35 0.75]);
yline( 10,'r--','LineWidth',1.6); yline(-10,'r--','LineWidth',1.6);
xline(2.0,'k:');
ylabel('\theta (deg)'); ylim([-12 12]);
title(sprintf('俯仰角：峰值 %.2f°（<10° 要求）', max(abs(o.th_deg(o.t>=2)))));

nexttile; hold on; grid on;
plot(o.t, o.x*1000, 'LineWidth',1.5, 'Color',[0.10 0.55 0.20]);
xline(2.0,'k:');
ylabel('位置 x (mm)'); xlabel('t (s)');
title(sprintf('位置：收敛到 %.1f mm', abs(o.x(end))*1000));

nexttile; hold on; grid on;
plot(o.t, o.v, 'LineWidth',1.4, 'Color',[0.85 0.15 0.15]);
xline(2.0,'k:');
ylabel('速度 (m/s)'); xlabel('t (s)');
title(sprintf('速度：收敛到 %.4f m/s', abs(o.v(end))));
savefig_png(f, fullfile(fig,'fig14_posfinal.png'));

%% ================= 4. 结论 =================
fprintf('\n================ 结论 ================\n');
fprintf('1. 位置环必须接在**目标俯仰角**上，不能接在腿部偏移上。\n');
m2 = R{2}.t >= R{2}.t_end-5;
fprintf('   旧接法(腿) 100s 后仍有 %.2f m 误差、末速 %.2f m/s 不归零；\n', ...
        max(abs(R{2}.x(m2))), max(abs(R{2}.v(m2))));
fprintf('   新接法(俯仰角) 收敛到 %.1f mm。\n', abs(o.x(end))*1000);
fprintf('2. 6050 的 +0.6° 硬偏置会产生 0.6/k 的稳态位置误差，必须归零。\n');
fprintf('3. 推荐 robot_kp = %.0f deg/m：峰值 %.1f°（<10°），收敛 %.1f mm。\n', ...
        k_best, max(abs(o.th_deg(o.t>=2))), abs(o.x(end))*1000);
fprintf('   真实质量(1.58kg)下最大可用 k≈8（峰值 9.7°），但收敛时间都在 ~35s，\n');
fprintf('   加大 k 只抬高瞬时倾角、不加快收敛，所以取 2 是性价比最优。\n');
fprintf('4. 收敛较慢(~40s)是因为速度环仍是纯 P；要更快需给速度环加积分。\n');
fprintf('\n图已保存到 %s\n', fig);

%% ====================== 局部函数 ==========================
function savefig_png(f, path)
exportgraphics(f, path, 'Resolution',130);
close(f);
end

function s = tern(c,a,b)
if c, s = a; else, s = b; end
end
