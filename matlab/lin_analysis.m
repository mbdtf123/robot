function R = lin_analysis(p, tau_max, pid)
%LIN_ANALYSIS  含执行器/传感器滞后 + 10ms 零阶保持的闭环稳定性分析
%
%   R = lin_analysis(p, tau_max, pid)
%
%   增广状态（8 维）
%     x1 位置      x2 速度          x3 俯仰角 rad   x4 俯仰角速度
%     x5 实际力矩  x6 俯仰角测量    x7 角速度测量   x8 编码器测速
%
%   对应源码中的滞后
%     x5 : 电流环 + 机械等效一阶      tau_m
%     x6 : Kalman + SMA 俯仰滞后      pitch_lag
%     x7 : 陀螺仪滞后                 gyro_lag
%     x8 : enc_update 速度低通        enc.tau
%
%   控制律（在 100 Hz 中断内按"度"为单位计算，与 main.cpp 完全一致）
%     target_pitch = vel_kp * v_meas                      [deg]
%     u            = kp*(target_pitch - pitch_meas) + kd*gyro_meas
%     tau_tot      = 2 * tau_max * u
%
%   返回 R.z（离散极点）、R.lam（连续极点）、R.stable、R.absz_max

if nargin < 3, pid = p.pid; end

m  = p.mech;
Ms = m.m_body + 2*m.m_wheel + 2*m.J_wheel/m.R^2;
Js = m.m_body*m.l^2 + m.J_body;
Mm = [Ms, -m.m_body*m.l; -m.m_body*m.l, Js];
Mi = inv(Mm);
gw = m.m_body*m.g*m.l;

B2 = Mi(1,1)/m.R + Mi(1,2);     % m/s^2  per N*m
B4 = Mi(2,1)/m.R + Mi(2,2);     % rad/s^2 per N*m

n = 8;
A = zeros(n);
A(1,2) = 1;
A(2,3) = Mi(1,2)*gw;   A(2,5) = B2;
A(3,4) = 1;
A(4,3) = Mi(2,2)*gw;   A(4,5) = B4;

A(5,5) = -1/p.motor.tau_m;
A(6,3) = 1/p.sens.pitch_lag;   A(6,6) = -1/p.sens.pitch_lag;
A(7,4) = 1/p.sens.gyro_lag;    A(7,7) = -1/p.sens.gyro_lag;
A(8,2) = 1/(p.mech.R*p.enc.tau);  A(8,8) = -1/p.enc.tau;

G = zeros(n,1);
G(5) = 1/p.motor.tau_m;         % 输入: 指令力矩

% ---- 反馈阵: tau_tot = K * x ----
K = zeros(1,n);
RAD = 180/pi;
% u = kp*(vel_kp*v_meas - pitch_meas*RAD) + kd*(gyro_meas*RAD)
K(6) = -2*tau_max*pid.balance_kp*RAD;      % 俯仰角反馈
K(7) =  2*tau_max*pid.balance_kd*RAD;      % 角速度反馈
K(8) =  2*tau_max*pid.balance_kp*pid.vel_kp; % 速度环 -> 目标俯仰角

Acl = A + G*K;

R.lam    = eig(Acl);
R.stable = all(real(R.lam) < 1e-9);

% ★ 正确做法: 先把"开环"离散化（ZOH），再在离散域闭环。
%   若先闭环再 c2d，得到的是 exp(lam*Ts)，永远稳定，会掩盖采样失稳。
sysD = c2d(ss(A, G, [], []), p.ctrl.Ts, 'zoh');
R.z    = eig(sysD.A + sysD.B*K);
R.absz = abs(R.z);
R.absz_max = max(R.absz);
R.d_stable = R.absz_max < 1.0;

% 采样裕度: 最快连续极点对应 |lam|*Ts
R.lam_fast = max(abs(real(R.lam)));
R.nyq_ratio = R.lam_fast * p.ctrl.Ts;   % >1 表示超出一个采样周期的尺度, 必然离散不稳
end
