function draw_robot(ax, art, p, st, VW, VH)
%DRAW_ROBOT  逐帧更新机器人简图（并联五连杆）
%
%   st.x   机身水平位置 (m)
%   st.th  俯仰角 (deg, 正 = 后倒)
%   st.legx/legy  固件下发的腿目标 (mm)
%   st.alive      是否仍在站立
%
%   机构（与 lib/Km/Kms.c 完全一致）
%     机架   O—O'            L5 = 62 mm，两舵机同一水平线
%     曲柄1  O→A             L1 = 70 mm，角 alpha
%     曲柄2  O'→C            L4 = 70 mm，角 beta
%     连杆1  A→B             L2 = 120 mm
%     连杆2  C→B             L3 = 120 mm
%     末端   B = 轮轴，目标坐标 (leg_x, leg_y)（腿坐标系, y 指向下方）
%
%   坐标变换
%     腿坐标系 y 向下，机身/世界坐标 y 向上，故映射时对 y 取负。
%     O 的世界位置由"轮轴 B 在世界中必须落在 (x, R)"反解得到。

R  = p.mech.R;
L1 = p.leg.L1/1000;  L2 = p.leg.L2/1000;  L3 = p.leg.L3/1000;
L4 = p.leg.L4/1000;  L5 = p.leg.L5/1000;

th  = st.th * pi/180;
Rth = [cos(th), -sin(th); sin(th), cos(th)];

% ---- 逆解得到两个曲柄角 ----
J  = inv_kinematics(st.legx, st.legy, st.legx, st.legy, p);
al = J.alpha_left;
be = J.beta_left;

% ---- 腿坐标系各节点 (m, y 向下) ----
B_l = [st.legx; st.legy] / 1000;
O_l = [0; 0];
O2_l = [L5; 0];
A_l = [L1*cos(al); L1*sin(al)];
C_l = [L5 + L4*cos(be); L4*sin(be)];

% ---- 世界坐标：先把腿坐标 y 翻成世界坐标，再旋转平移 ----
P = [B_l(1); -B_l(2)];                  % B 相对 O（y 向上）
Bh = [st.x; R];                         % 轮轴世界位置
O  = Bh - Rth*P;                        % 舵机1 的机身位置
toW = @(pt) O + Rth*[pt(1); -pt(2)];
Bw = toW(B_l);
Aw = toW(A_l);
Cw = toW(C_l);
O2w = toW(O2_l);

% ---- 车轮 + 轮辐 ----
tc = linspace(0, 2*pi, 49);
set(art.wheel, 'XData', Bw(1)+R*cos(tc), 'YData', Bw(2)+R*sin(tc));
phi = st.x / R;                          % 纯滚动
for s = 1:2
    ph = phi + (s-1)*pi/2;
    hh = art.sp1; if s == 2, hh = art.sp2; end
    set(hh, 'XData', [Bw(1), Bw(1)+R*cos(ph)], ...
            'YData', [Bw(2), Bw(2)+R*sin(ph)]);
end

% ---- 五连杆 ----
set(art.frame,  'XData', [O(1) O2w(1)],  'YData', [O(2) O2w(2)]);
set(art.crank1, 'XData', [O(1) Aw(1)],   'YData', [O(2) Aw(2)]);
set(art.crank2, 'XData', [O2w(1) Cw(1)], 'YData', [O2w(2) Cw(2)]);
set(art.link1,  'XData', [Aw(1) Bw(1)],  'YData', [Aw(2) Bw(2)]);
set(art.link2,  'XData', [Cw(1) Bw(1)],  'YData', [Cw(2) Bw(2)]);
set(art.pivot1, 'XData', O(1),   'YData', O(2));
set(art.pivot2, 'XData', O2w(1), 'YData', O2w(2));

% ---- 机身（装在机架上方，随 θ 旋转）----
% 机架中心为机身基准
B0 = (O + O2w)/2;
B  = [-0.052 0.012; 0.052 0.012; 0.052 0.150; -0.052 0.150];
Bw2 = (B0 + (Rth*B.')).';
set(art.body, 'XData', Bw2(:,1), 'YData', Bw2(:,2));

S  = [-0.020 0.150; 0.020 0.150; 0.020 0.180; -0.020 0.180];
Sw = (B0 + (Rth*S.')).';
set(art.sensor, 'XData', Sw(:,1), 'YData', Sw(:,2));

% ---- 文字 ----
if st.alive
    set(art.info, 'String', sprintf('\\theta = %+.2f^o   \\alpha = %.1f^o   \\beta = %.1f^o', ...
        st.th, al*180/pi, be*180/pi), 'Position', [st.x, 0.368]);
else
    set(art.info, 'String', sprintf('\\theta = %+.2f^o  (倒地)', st.th), ...
        'Position', [st.x, 0.368]);
end

% ---- 相机跟随 ----
set(ax, 'XLim', [st.x-VW/2, st.x+VW/2], 'YLim', [-0.03, VH]);
set(art.ground, 'XData', [st.x-2, st.x+2]);
end
