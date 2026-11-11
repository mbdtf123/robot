%% ANIM_ROBOT  2D 动画回放：原样固件跌倒 vs 修复后站稳
%
%   用法: 在 matlab/ 目录下运行  anim_robot
%   输出: anim/compare.mp4  (60 fps)
%         anim/compare.gif  (25 fps, 可直接双击预览)
%         anim/preview.png  (首帧静图)
%
%   画面
%     左：复刻 src/main.cpp 原样参数 + 350ms 阻塞式舵机  -> 0.22s 倒地
%     右：vel_kp 改号 + kd 降档 + 舵机异步              -> 稳定站立
%     下：俯仰角 / 力矩指令 的实时曲线
%
%   机器人简图说明
%     腿按固件常量 L1=70 / L2=120 的二连杆绘制，膝角解与
%     Kms.c 输出的 alpha 完全一致（站立点 alpha=155.21°）；
%     轮轴相对髋的位置直接取固件下发的 (leg_x, leg_y)，
%     因此防倒机构推动腿部时，画面上的腿会真实地屈伸。

clc; close all;
p = robot_params();
outdir = fullfile(pwd, 'anim');
if ~exist(outdir, 'dir'), mkdir(outdir); end

%% ================= 两段仿真 =================
pA = p;  pA.sim.fall_deg = 100;                 % 让它真的倒到躺下
scA = struct('name','原样','T',2.5,'noise',false,'leg',true,'block',true);
A = sim_balance(pA, scA);

pB = p;
pB.pid.vel_kp     = +0.05;
pB.pid.balance_kd = -0.01;
scB = struct('name','修复','T',2.5,'noise',true,'leg',true,'block',false);
B = sim_balance(pB, scB);

fprintf('左侧(原样): 倒地 %.3f s | 右侧(修复): theta RMS %.2f deg\n', A.t_end, B.th_rms);

%% ================= 画面设置 =================
T   = 2.5;
fps = 60;
nf  = round(T*fps);

f = figure('Visible','off', 'Position',[60 60 1320 780], 'Color','w');

axL = axes(f, 'Position',[0.045 0.45 0.375 0.45]); hold(axL,'on');
axR = axes(f, 'Position',[0.545 0.45 0.375 0.45]); hold(axR,'on');
axT = axes(f, 'Position',[0.070 0.075 0.40 0.27]); hold(axT,'on'); grid(axT,'on'); box(axT,'off');
axU = axes(f, 'Position',[0.555 0.075 0.40 0.27]); hold(axU,'on'); grid(axU,'on'); box(axU,'off');

% 两个机器人面板同一视场
VW = 0.62;  VH = 0.46;
set(axL,'DataAspectRatio',[1 1 1]);
set(axR,'DataAspectRatio',[1 1 1]);

% 曲线：预先画完整轨迹，用移动标记指示当前时刻
hTL = plot(axT, A.t, A.th_deg, 'LineWidth',1.0, 'Color',[0.85 0.15 0.15]); hold(axT,'on');
hTR = plot(axT, B.t, B.th_deg, 'LineWidth',1.0, 'Color',[0.10 0.55 0.20]);
yline(axT, 0, 'k-', 'LineWidth',0.4);
yline(axT, 10,'k:','LineWidth',0.6); yline(axT,-10,'k:','LineWidth',0.6);
xlim(axT,[0 T]); ylim(axT,[-108 30]); ylabel(axT,'\theta (deg)'); xlabel(axT,'t (s)');

hUL = plot(axU, A.t, A.u, 'LineWidth',0.9, 'Color',[0.85 0.15 0.15]); hold(axU,'on');
hUR = plot(axU, B.t, B.u, 'LineWidth',0.9, 'Color',[0.10 0.55 0.20]);
yline(axU, [1 -1], 'r--', 'LineWidth',0.6);
xlim(axU,[0 T]); ylim(axU,[-1.35 1.35]); ylabel(axU,'u [-]'); xlabel(axU,'t (s)');

% 移动标记
mTL = plot(axT, 0, 0, 'o', 'MarkerSize',7, 'MarkerFaceColor',[0.85 0.15 0.15],'MarkerEdgeColor','none');
mTR = plot(axT, 0, 0, 'o', 'MarkerSize',7, 'MarkerFaceColor',[0.10 0.55 0.20],'MarkerEdgeColor','none');
mUL = plot(axU, 0, 0, 'o', 'MarkerSize',7, 'MarkerFaceColor',[0.85 0.15 0.15],'MarkerEdgeColor','none');
mUR = plot(axU, 0, 0, 'o', 'MarkerSize',7, 'MarkerFaceColor',[0.10 0.55 0.20],'MarkerEdgeColor','none');

% 预留的动画图元容器
artL = init_robot_axes(axL, p, '原样固件（并联五连杆 + 350 ms 阻塞舵机）', [0.85 0.15 0.15]);
artR = init_robot_axes(axR, p, '修复后（并联五连杆 + vel_kp 改号 + kd 降档）', [0.10 0.55 0.20]);

% 倒计时 / 计时文字
hClock = annotation(f,'textbox',[0.40 0.935 0.20 0.045], 'String','t = 0.00 s', ...
    'HorizontalAlignment','center','FontSize',13,'FontWeight','bold', ...
    'EdgeColor','none','Interpreter','none');

% 视频
vw = VideoWriter(fullfile(outdir,'compare.mp4'), 'MPEG-4');
vw.FrameRate = fps; vw.Quality = 92; open(vw);
gifpath = fullfile(outdir,'compare.gif');

fprintf('开始渲染 %d 帧 ...\n', nf);
for k = 1:nf
    t = (k-1)/fps;

    % ---- 左：原样（倒地后定格） ----
    iA = find(A.t <= t, 1, 'last');  if isempty(iA), iA = 1; end
    if t <= A.t_end
        stA.x = A.x(iA);  stA.th = A.th_deg(iA);
        stA.legx = p.leg.stand_x + A.leg_off(iA);  stA.legy = p.leg.stand_y;
        stA.u = A.u(iA);  stA.alive = true;
    else
        stA.x = A.x(end); stA.th = A.th_deg(end);
        stA.legx = p.leg.stand_x + A.leg_off(end); stA.legy = p.leg.stand_y;
        stA.u = A.u(end); stA.alive = false;
    end
    draw_robot(axL, artL, p, stA, VW, VH);

    % ---- 右：修复后 ----
    iB = find(B.t <= t, 1, 'last');  if isempty(iB), iB = 1; end
    stB.x = B.x(iB);  stB.th = B.th_deg(iB);
    stB.legx = p.leg.stand_x + B.leg_off(iB);  stB.legy = p.leg.stand_y;
    stB.u = B.u(iB);  stB.alive = true;
    draw_robot(axR, artR, p, stB, VW, VH);

    % ---- 状态文字 ----
    if t <= A.t_end
        set(artL.status, 'String', '平衡中', 'Color', [0.85 0.15 0.15]);
    else
        set(artL.status, 'String', sprintf('✗ 跌倒 @ %.2f s', A.t_end), ...
            'Color', [0.85 0.15 0.15]);
    end
    set(artR.status, 'String', '✓ 稳定站立', 'Color', [0.10 0.55 0.20]);
    set(artL.status, 'Position', [stA.x, 0.405]);
    set(artR.status, 'Position', [stB.x, 0.405]);

    % ---- 曲线标记 ----
    set(mTL,'XData',A.t(iA),'YData',A.th_deg(iA));
    set(mTR,'XData',B.t(iB),'YData',B.th_deg(iB));
    set(mUL,'XData',A.t(iA),'YData',A.u(iA));
    set(mUR,'XData',B.t(iB),'YData',B.u(iB));
    hClock.String = sprintf('t = %.2f s', t);

    drawnow;
    fr = getframe(f);
    writeVideo(vw, fr);

    % GIF 抽帧到 25 fps，且缩到一半分辨率
    if mod(k-1, max(round(fps/25),1)) == 0
        img = fr.cdata(1:2:end, 1:2:end, :);
        [im8, map] = rgb2ind(img, 256);
        if k == 1
            imwrite(im8, map, gifpath, 'gif', 'LoopCount',inf, 'DelayTime',0.04);
        else
            imwrite(im8, map, gifpath, 'gif', 'WriteMode','append', 'DelayTime',0.04);
        end
    end
    if mod(k, 30) == 0, fprintf('  %d/%d\n', k, nf); end
end
close(vw);
% legend 必须放在所有 plot 之后，否则会被后建的标记点污染成 4 项
legend(axT, [hTL hTR], {'原样固件','修复后'}, 'Location','southeast','Box','off');
exportgraphics(f, fullfile(outdir,'preview.png'), 'Resolution',120);
close(f);

fprintf('完成:\n  %s\n  %s\n', fullfile(outdir,'compare.mp4'), gifpath);
