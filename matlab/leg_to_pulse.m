function [pulse, info] = leg_to_pulse(alpha, beta, side, p)
%LEG_TO_PULSE  关节角 -> 舵机脉冲（src/main.cpp:259-337 set_leg_position_smooth 的移植）
%
%   [pulse, info] = leg_to_pulse(alpha, beta, side, p)
%
%   输入 alpha, beta 单位 rad（直接来自 inv_kinematics）
%        side 取 'L' 或 'R'
%   输出 pulse = [大腿脉冲, 小腿脉冲]
%
%   移植时保留了源码中左右两侧 *不一致* 的限幅（见 robot_params 第 7 节），
%   这不是笔误，是原样复现；副作用见 README「发现 6」。

alpha_deg = alpha * 180/pi;
beta_deg  = beta  * 180/pi;

% ---- 角度限幅（main.cpp:277-280）----
alpha_deg = min(max(alpha_deg, p.leg.alpha_lim(1)), p.leg.alpha_lim(2));
beta_deg  = min(max(beta_deg , p.leg.beta_lim(1)),  p.leg.beta_lim(2));

% ---- 角度偏移（main.cpp:283-284）----
s0 = alpha_deg - 90;
s2 = 90 - beta_deg;

% ---- 舵机旋转范围限幅（左右不同, main.cpp:287-290 / 306-309）----
if side == 'L'
    s0 = min(max(s0, p.leg.s0_lim_L(1)), p.leg.s0_lim_L(2));
    s2 = min(max(s2, p.leg.s2_lim_L(1)), p.leg.s2_lim_L(2));
    off0 = p.leg.off_LF;
    off2 = p.leg.off_LR;
else
    s0 = min(max(s0, p.leg.s0_lim_R(1)), p.leg.s0_lim_R(2));
    s2 = min(max(s2, p.leg.s2_lim_R(1)), p.leg.s2_lim_R(2));
    off0 = p.leg.off_RR;
    off2 = p.leg.off_RF;
end

% ---- 脉冲换算（main.cpp:299-300）----
pl0 = p.leg.base_pulse + off0 + s0 * p.leg.pulse_per_deg;
pl2 = p.leg.base_pulse + off2 + s2 * p.leg.pulse_per_deg;

% ---- 脉冲限幅（main.cpp:315-322）----
pl0 = min(max(round(pl0), p.leg.pulse_lim(1)), p.leg.pulse_lim(2));
pl2 = min(max(round(pl2), p.leg.pulse_lim(1)), p.leg.pulse_lim(2));

pulse = [pl0, pl2];
if nargout > 1
    info.alpha_deg = alpha_deg;
    info.beta_deg  = beta_deg;
    info.s0        = s0;
    info.s2        = s2;
end
end
