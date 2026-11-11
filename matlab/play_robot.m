function play_robot(speed, mode)
%PLAY_ROBOT  实时窗口动画（不生成任何视频文件）
%
%   play_robot               受力演示（默认）：被推一把，看能否恢复
%   play_robot(2.0)          2 倍速
%   play_robot(1.0,'compare') 原样固件 vs 修复后（一上电就倒的对比）
%
%   受力模式画面
%     左：修复后参数但**关掉位置环**  -> 被推后一直滚走
%     右：修复后参数 + **位置环开**    -> 被推后自己退回来
%     推力用红色箭头 + 读数实时显示
%
%   窗口出现后：按任意键提前结束；关闭窗口即停止。

if nargin < 1 || isempty(speed), speed = 2.0; end
if nargin < 2 || isempty(mode),  mode  = 'force'; end

p = robot_params();
q = p;
q.pid.vel_kp     = +0.05;
q.pid.balance_kd = -0.01;
q.sens.pitch_bias = 0.0;

forcemode = strcmpi(mode, 'force');

%% ---------- 仿真 ----------
if forcemode
    T       = 10;          % 仿真时长
    DIST_T  = 2.0;         % 推力时刻
    DIST_J  = 1.5;         % 冲量 N·s
    DIST_D = 0.05;         % 持续
    DIST_F = DIST_J / DIST_D;

    scA = struct('T',T,'noise',true,'leg',true,'block',false,'tau_max',0.12, ...
                 'dist_on',true,'dist_t',DIST_T,'dist_F',DIST_F,'dist_dur',DIST_D, ...
                 'pos_on',false);
    A = sim_balance(q, scA);

    scB = scA;  scB.pos_on = true;  scB.pos_mode = 'pitch';  scB.k_pos = 2;  scB.pos_target = 0;
    B = sim_balance(q, scB);

    ttlL = sprintf('关掉位置环（robot_kp=0）  被推后一直滚走');
    ttlR = sprintf('位置环开（robot_kp=2）  被推后自己退回');
    yLimCurve = [-6 6];
else
    T = 2.5;
    pA = p; pA.sim.fall_deg = 100;
    A  = sim_balance(pA, struct('T',T,'noise',false,'leg',true,'block',true));

    pB = p; pB.pid.vel_kp = +0.05; pB.pid.balance_kd = -0.01;
    B  = sim_balance(pB, struct('T',T,'noise',true,'leg',true,'block',false));

    ttlL = '原样固件（并联五连杆 + 350 ms 阻塞舵机）';
    ttlR = '修复后（vel_kp 改号 + kd 降档 + 舵机异步）';
    yLimCurve = [-108 30];
end

%% ---------- 窗口 ----------
f = figure('Name','轮足机器人 —— 实时动画', 'NumberTitle','off', ...
    'Position',[60 60 1340 790], 'Color','w', ...
    'KeyPressFcn', @(s,~) setappdata(s,'stop',true));
setappdata(f,'stop',false);

axL = axes(f, 'Position',[0.040 0.44 0.375 0.47]); hold(axL,'on');
axR = axes(f, 'Position',[0.545 0.44 0.375 0.47]); hold(axR,'on');
axT = axes(f, 'Position',[0.070 0.065 0.395 0.28]); hold(axT,'on'); grid(axT,'on'); box(axT,'off');
axV = axes(f, 'Position',[0.560 0.065 0.395 0.28]); hold(axV,'on'); grid(axV,'on'); box(axV,'off');

set(axL,'DataAspectRatio',[1 1 1]);
set(axR,'DataAspectRatio',[1 1 1]);
VW = 0.62;  VH = 0.46;

% ---- 曲线（完整轨迹 + 移动标记）----
hTL = plot(axT, A.t, A.th_deg, 'LineWidth',1.0, 'Color',[0.85 0.15 0.15]);
hTR = plot(axT, B.t, B.th_deg, 'LineWidth',1.0, 'Color',[0.10 0.55 0.20]);
yline(axT, 0, 'k-', 'LineWidth',0.4);
if forcemode
    yline(axT, 10, 'r--', 'LineWidth',1.2, 'Alpha',0.5);
    yline(axT,-10, 'r--', 'LineWidth',1.2, 'Alpha',0.5);
end
xlim(axT,[0 T]); ylim(axT, yLimCurve);
ylabel(axT,'\theta (deg)'); xlabel(axT,'t (s)');

plot(axV, A.t, A.v, 'LineWidth',1.0, 'Color',[0.85 0.15 0.15]);
plot(axV, B.t, B.v, 'LineWidth',1.0, 'Color',[0.10 0.55 0.20]);
yline(axV, 0, 'k-', 'LineWidth',0.4);
xlim(axV,[0 T]);
ylabel(axV,'速度 v (m/s)'); xlabel(axV,'t (s)');

% ---- 冲击时刻竖线 ----
if forcemode
    xline(axT, DIST_T, 'r:', 'LineWidth',1.2);
    xline(axV, DIST_T, 'r:', 'LineWidth',1.2);
    text(axT, DIST_T, yLimCurve(2)*0.92, sprintf('  推力 %.0f N / %.2f N·s', DIST_F, DIST_J), ...
        'Color',[0.85 0.15 0.15], 'FontSize',9, 'FontWeight','bold', 'VerticalAlignment','top');
end

mTL = plot(axT, 0, 0, 'o', 'MarkerSize',7, 'MarkerFaceColor',[0.85 0.15 0.15],'MarkerEdgeColor','none');
mTR = plot(axT, 0, 0, 'o', 'MarkerSize',7, 'MarkerFaceColor',[0.10 0.55 0.20],'MarkerEdgeColor','none');
mVL = plot(axV, 0, 0, 'o', 'MarkerSize',7, 'MarkerFaceColor',[0.85 0.15 0.15],'MarkerEdgeColor','none');
mVR = plot(axV, 0, 0, 'o', 'MarkerSize',7, 'MarkerFaceColor',[0.10 0.55 0.20],'MarkerEdgeColor','none');

% ---- 机器人 ----
artL = init_robot_axes(axL, p, ttlL, [0.85 0.15 0.15]);
artR = init_robot_axes(axR, p, ttlR, [0.10 0.55 0.20]);

% ---- 推力箭头（默认隐藏）----
aR = [0.85 0.10 0.10];
hArrowL = quiver(axL, 0, 0, 0, 0, 0, 'Color',aR, 'LineWidth',3, 'MaxHeadSize',0.8, 'Visible','off');
hArrowR = quiver(axR, 0, 0, 0, 0, 0, 'Color',aR, 'LineWidth',3, 'MaxHeadSize',0.8, 'Visible','off');
hForceL = text(axL, 0, 0, '', 'Color',aR, 'FontSize',11, 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'Visible','off');
hForceR = text(axR, 0, 0, '', 'Color',aR, 'FontSize',11, 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'Visible','off');

% legend 放在所有 plot 之后
legend(axT, [hTL hTR], {'无位置环','有位置环'}, 'Location','southwest','Box','off');

hClock = annotation(f,'textbox',[0.40 0.945 0.20 0.04], 'String','t = 0.00 s', ...
    'HorizontalAlignment','center','FontSize',13,'FontWeight','bold', ...
    'EdgeColor','none','Interpreter','none');
hHint = annotation(f,'textbox',[0.28 0.005 0.44 0.035], ...
    'String','按任意键停止 / 关闭窗口退出', 'HorizontalAlignment','center', ...
    'FontSize',9,'EdgeColor','none','Color',[0.5 0.5 0.5],'Interpreter','none');

%% ---------- 实时播放 ----------
fps = 50;
nf  = round(T*fps);
t0  = tic;

for k = 1:nf
    if ~ishandle(f) || getappdata(f,'stop'), break; end
    t = (k-1)/fps;

    % ---- 左 ----
    iA = find(A.t <= t, 1, 'last');  if isempty(iA), iA = 1; end
    stA.x = A.x(iA);  stA.th = A.th_deg(iA);
    stA.legx = p.leg.stand_x + A.leg_off(iA);  stA.legy = p.leg.stand_y;
    stA.alive = true;
    if forcemode
        stA.alive = t <= A.t_end;
    end
    draw_robot(axL, artL, p, stA, VW, VH);

    % ---- 右 ----
    iB = find(B.t <= t, 1, 'last');  if isempty(iB), iB = 1; end
    stB.x = B.x(iB);  stB.th = B.th_deg(iB);
    stB.legx = p.leg.stand_x + B.leg_off(iB);  stB.legy = p.leg.stand_y;
    stB.alive = true;
    draw_robot(axR, artR, p, stB, VW, VH);

    % ---- 状态文字 ----
    if forcemode
        set(artL.status, 'String', '无位置环：滚走', 'Color', [0.85 0.15 0.15]);
        set(artR.status, 'String', '有位置环：回位', 'Color', [0.10 0.55 0.20]);
    elseif t <= A.t_end
        set(artL.status, 'String', '平衡中', 'Color', [0.85 0.15 0.15]);
    else
        set(artL.status, 'String', sprintf('✗ 跌倒 @ %.2f s', A.t_end), 'Color', [0.85 0.15 0.15]);
    end
    if ~forcemode
        set(artR.status, 'String', '✓ 稳定站立', 'Color', [0.10 0.55 0.20]);
    end
    set(artL.status, 'Position', [stA.x, 0.405]);
    set(artR.status, 'Position', [stB.x, 0.405]);

    % ---- 推力箭头（受力窗口内显示，略加宽以便看清）----
    if forcemode
        showForce = (t >= DIST_T - 0.05) && (t <= DIST_T + 0.30);
        if showForce
            % 箭头从机器人左后方指向机身：体现"从后面推"
            ax0 = stA.x - 0.285;  ay0 = 0.165;
            ax1 = stB.x - 0.285;  ay1 = 0.165;
            set(hArrowL,'XData',ax0,'YData',ay0,'UData',0.115,'VData',0, 'Visible','on');
            set(hArrowR,'XData',ax1,'YData',ay1,'UData',0.115,'VData',0, 'Visible','on');
            set(hForceL,'String',sprintf('F = %.0f N', DIST_F), ...
                'Position',[ax0+0.06, 0.245], 'Visible','on');
            set(hForceR,'String',sprintf('F = %.0f N', DIST_F), ...
                'Position',[ax1+0.06, 0.245], 'Visible','on');
        else
            set(hArrowL,'Visible','off'); set(hArrowR,'Visible','off');
            set(hForceL,'Visible','off'); set(hForceR,'Visible','off');
        end
    end

    % ---- 曲线标记 ----
    set(mTL,'XData',A.t(iA),'YData',A.th_deg(iA));
    set(mTR,'XData',B.t(iB),'YData',B.th_deg(iB));
    set(mVL,'XData',A.t(iA),'YData',A.v(iA));
    set(mVR,'XData',B.t(iB),'YData',B.v(iB));
    hClock.String = sprintf('t = %.2f s', t);

    drawnow;

    while toc(t0) < t / speed
        pause(0.002);
        if ~ishandle(f) || getappdata(f,'stop'), break; end
    end
end

if ishandle(f)
    if ishandle(hHint), delete(hHint); end
    annotation(f,'textbox',[0.22 0.005 0.56 0.035], ...
        'String','播放结束 —— 关闭窗口，或再次运行 play_robot 重播', ...
        'HorizontalAlignment','center','FontSize',9,'EdgeColor','none', ...
        'Color',[0.5 0.5 0.5],'Interpreter','none');
end
end
