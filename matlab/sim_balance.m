function out = sim_balance(p, sc)
%SIM_BALANCE  轮足平衡机器人闭环仿真（复刻 src/main.cpp 的控制结构）
%
%   out = sim_balance(p, sc)
%
%   sc 字段（全部可选）
%     .name      场景名（仅用于打印）
%     .T         仿真时长 s            默认 p.sim.T
%     .noise     传感器噪声开关       默认 false
%     .leg       腿部防倒逻辑开关     默认 true
%     .block     true = 复刻 set_leg_position_smooth 的 350ms 阻塞
%                                     默认 false（异步舵机的理想情况）
%     .gyro_sign 陀螺仪符号 +1/-1     默认 +1
%     .tau_max   单轮峰值转矩覆盖     默认 p.motor.tau_max
%     .vel_tgt   目标速度 (rad/s)     默认 0
%     .theta0    初始俯仰角 deg       默认 p.sim.theta0
%     .pid       PID 参数覆盖         默认 p.pid
%     .leg_ctrl  防倒参数覆盖         默认 p.leg_ctrl
%
%   与源码一一对应的环节
%     * 100 Hz 控制中断（main.cpp:218 CONTROL_INTERVAL_MS=10）
%     * 双环 PID（main.cpp:442-466 calculate_wheel_torque）
%     * 腿部防倒（main.cpp:380-439 adjust_leg_position_for_balance）
%     * 阻塞式舵机（main.cpp:331 vTaskDelay(duration+100)）
%     * 编码器测速一阶低通（esp_hw_link.cpp:110 alpha=0.3）
%     * 力矩一阶动态（电流环等效）
%     * MPU6050 姿态滞后 + +0.6 度硬偏置（6050_middle.cpp:310）

if nargin < 2, sc = struct(); end
sc = fill_sc(sc, p);

h  = p.sim.h;
Ts = p.ctrl.Ts;

%% ---------- 状态初始化 ----------
x      = [0; 0; deg2rad(sc.theta0); 0];  % [位置; 速度; 俯仰(rad,+后倒); 角速度]
tau_m  = 0;        % 实际单轮转矩 (N*m)
th_f   = sc.theta0;% 俯仰角滤波值 (deg)
g_f    = 0;        % 陀螺仪滤波值 (deg/s)
v_f    = 0;        % 编码器速度滤波值 (rad/s)
tau_cmd = 0;       % 控制器输出 [-1,1]
leg_off = 0;       % 防倒腿偏移 (mm)
leg_tgt = p.leg.stand_x;   % 腿目标 X (mm)
leg_act = p.leg.stand_x;   % 腿实际 X (mm)，舵机 250ms 行程
next_ctrl   = 0;
block_until = -inf;
n_trig      = 0;
blocked_tot = 0;

a_pitch = 1 - exp(-h/p.sens.pitch_lag);
a_gyro  = 1 - exp(-h/p.sens.gyro_lag);
a_vel   = 1 - exp(-h/p.enc.tau);
a_tau   = 1 - exp(-h/p.motor.tau_m);
a_leg   = 1 - exp(-h/max(p.leg.move_ms/3000, h));   % 舵机行程等效一阶

Nmax = ceil(sc.T/h) + 100;
rec  = nan(Nmax, 12);
t = 0;  k = 0;  fallen = false;
tgt_now = 0;    % 最近一次控制算出的目标俯仰角（仅用于记录）

%% ---------- 主循环 ----------
while t <= sc.T + 1e-12 && ~fallen && k < Nmax
    % ================= 控制中断（每 10 ms）=================
    if t >= next_ctrl - 1e-12 && t >= block_until - 1e-12
        % ---- 读传感器 ----
        th_meas = th_f + p.sens.pitch_bias ...
                  + sc.noise * p.sens.pitch_noise * randn();
        g_meas  = g_f + sc.noise * (p.sens.gyro_bias + p.sens.gyro_noise * randn());
        v_meas  = v_f;

        % ---- 双环 PID（main.cpp:442-466）----
        vel_err   = v_meas - sc.vel_tgt;
        tgt_pitch = clamp(sc.pid.vel_kp * vel_err, -p.pid.pitch_lim, p.pid.pitch_lim);

        % ---- 位置环 [方案B]：位置误差 -> 目标俯仰角（推荐接法）----
        if sc.pos_on && strcmp(sc.pos_mode, 'pitch')
            tgt_pitch = tgt_pitch + sc.k_pos * (x(1) - sc.pos_target);
            tgt_pitch = clamp(tgt_pitch, -p.pid.pitch_lim, p.pid.pitch_lim);
        end

        ang_err   = tgt_pitch - th_meas;
        u         = sc.pid.balance_kp*ang_err + sc.pid.balance_kd*g_meas;
        tau_cmd   = clamp(u, -p.ctrl.max_torque, p.ctrl.max_torque);
        tgt_now   = tgt_pitch;
        next_ctrl = t + Ts;

        % ---- 腿部防倒（main.cpp:380-439）----
        if sc.leg
            leg_off = leg_control(th_meas, sc.leg_ctrl);

            % ---- 位置环 [方案A]：位置误差 -> 腿部偏移（main.cpp 现有实现）----
            if sc.pos_on && strcmp(sc.pos_mode, 'leg')
                leg_off = clamp(leg_off + sc.k_pos * (x(1) - sc.pos_target), ...
                                -sc.leg_ctrl.max_off, sc.leg_ctrl.max_off);
            end

            new_x   = clamp(p.leg.stand_x + leg_off, 0, 100);
            if abs(new_x - leg_tgt) > sc.leg_ctrl.trig_mm
                n_trig  = n_trig + 1;
                leg_tgt = new_x;
                if sc.block
                    % ★ set_leg_position_smooth 内部 vTaskDelay(250+100)
                    %   整个平衡环被冻结 350 ms：不发新力矩、不跑 motor_update
                    block_until = t + p.leg.block_s;
                    next_ctrl   = block_until + Ts;
                    blocked_tot = blocked_tot + p.leg.block_s;
                end
            end
        end
    end

    % ================= 记录 =================
    k = k + 1;
    rec(k, :) = [t, x(1), x(2), rad2deg(x(3)), rad2deg(x(4)), ...
                 th_f + p.sens.pitch_bias, tau_cmd, tau_m, leg_off, leg_act, ...
                 double(t < block_until), tgt_now];

    % ================= 传感器滤波（等效一阶滞后）=================
    th_f = th_f + (rad2deg(x(3)) - th_f) * a_pitch;
    g_f  = g_f  + (sc.gyro_sign*rad2deg(x(4)) - g_f) * a_gyro;
    v_f  = v_f  + (x(2)/p.mech.R - v_f) * a_vel;

    % ================= 执行器 / 腿部行程 =================
    tau_m   = tau_m   + (tau_cmd*sc.tau_max - tau_m) * a_tau;   % 阻塞期间 tau_cmd 冻结
    leg_act = leg_act + (leg_tgt - leg_act) * a_leg;

    % ================= 刚体动力学（RK4）=================
    dx_cm = -(leg_act - p.leg.stand_x)/1000 * p.mech.leg_coupling;

    % ---- 外力扰动（推一把）: t∈[dist_t, dist_t+dist_dur) 内施加恒力 ----
    if sc.dist_on && t >= sc.dist_t && t < (sc.dist_t + sc.dist_dur)
        F_ext = sc.dist_F;
    else
        F_ext = 0;
    end

    x = rk4_step(t, x, 2*tau_m, p, dx_cm, h, F_ext);
    t = t + h;

    % ---- 跌倒判定 ----
    if abs(rad2deg(x(3))) > p.sim.fall_deg
        fallen = true;
        k = k + 1;
        rec(k, :) = [t, x(1), x(2), rad2deg(x(3)), rad2deg(x(4)), th_f, ...
                     tau_cmd, tau_m, leg_off, leg_act, 0, 0];
    end
end

rec = rec(1:k, :);

%% ---------- 结果整理 ----------
out.name    = sc.name;
out.t       = rec(:,1);
out.x       = rec(:,2);
out.v       = rec(:,3);
out.th_deg  = rec(:,4);
out.w_deg   = rec(:,5);
out.th_meas = rec(:,6);
out.u       = rec(:,7);
out.tau     = rec(:,8);
out.leg_off = rec(:,9);
out.leg_act = rec(:,10);
out.blocked = rec(:,11);
out.tgt_pitch = rec(:,12);

out.fallen   = fallen;
out.n_trig   = n_trig;
out.t_end    = rec(end,1);
out.blocked_tot = min(blocked_tot, out.t_end);      % 实际被冻结的时间 (s)
out.block_frac  = out.blocked_tot / max(out.t_end, eps);
out.sc       = sc;

% 稳定裕度指标：跳过前 30%，看后段的俯仰角 RMS 与峰值
i0 = max(1, round(0.3*numel(out.t)));
seg = out.th_deg(i0:end);
out.th_rms = sqrt(mean(seg.^2));
out.th_max = max(abs(seg));
out.stable = ~fallen && out.th_rms < 5 && out.th_max < 15;
end

%% ========================================================================
function sc = fill_sc(sc, p)
d.name      = 'scenario';
d.T         = p.sim.T;
d.noise     = false;
d.leg       = true;
d.block     = false;
d.gyro_sign = +1;
d.tau_max   = p.motor.tau_max;
d.vel_tgt   = 0;
d.theta0    = p.sim.theta0;
d.pid       = p.pid;
d.leg_ctrl  = p.leg_ctrl;
% ---- 外力扰动（默认关闭）----
d.dist_on   = false;   % 是否施加扰动
d.dist_t    = 2.0;     % 扰动开始时刻 (s)
d.dist_F    = 0;       % 水平外力 (N), 正 = 向前推车身
d.dist_dur  = 0.05;    % 扰动持续 (s)
% ---- 位置环（默认关闭）----
d.pos_on    = false;
d.pos_mode  = 'pitch'; % 'pitch' = 误差加到目标俯仰角（推荐）; 'leg' = 加到腿部偏移（原代码）
d.k_pos     = 0;       % pitch 模式单位 deg/m; leg 模式单位 mm/m
d.pos_target = 0;      % 目标位置 (m)
fn = fieldnames(d);
for i = 1:numel(fn)
    if ~isfield(sc, fn{i}), sc.(fn{i}) = d.(fn{i}); end
end
end

%% ========================================================================
function off = leg_control(pitch_meas, lc)
% 主控制逻辑：main.cpp:391-407 —— 后倒后移、前倾前移，带死区与限幅
if pitch_meas > lc.dead_zone
    off = -abs(pitch_meas) * lc.gain;
elseif pitch_meas < -lc.dead_zone
    off =  abs(pitch_meas) * lc.gain;
else
    off = 0;
end
off = clamp(off, -lc.max_off, lc.max_off);
end

%% ========================================================================
function x = rk4_step(t, x, tau_tot, p, dx_cm, h, F_ext)
f  = @(tt, xx) wip_dynamics(tt, xx, tau_tot, p, dx_cm, F_ext);
k1 = f(t,        x);
k2 = f(t + h/2,  x + h/2*k1);
k3 = f(t + h/2,  x + h/2*k2);
k4 = f(t + h,    x + h*k3);
x  = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

%% ========================================================================
function v = clamp(v, lo, hi)
v = min(max(v, lo), hi);
end
