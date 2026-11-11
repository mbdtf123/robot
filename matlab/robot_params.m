function p = robot_params()
%ROBOT_PARAMS  轮足机器人 MATLAB 模型参数（源自本工程源码）
%
%   标记 [CODE]   : 直接取自工程源码，附 文件:行号
%   标记 [ASSUME] : 源码中不存在该量，取典型值；需实测后替换再跑 run_model
%
%   源码依据
%     src/main.cpp                          主控制逻辑 / PID / 站立几何
%     lib/Km/Kms.h  lib/Km/Kms.c            运动学连杆长度 L1~L5
%     lib/FOC/foc_config.h                  电机 / 编码器 / 电压
%     lib/FOC/esp_hw_link.cpp               力矩->电压、编码器测速
%     lib/6050mpu/6050_middle.cpp           姿态解算（Kalman + SMA）
%
%   用法:  p = robot_params();

p.meta.version = '1.0';
p.meta.date    = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));

%% ================= 1. 控制周期与限幅 =================
p.ctrl.Ts          = 10e-3;    % [CODE] main.cpp:39   CONTROL_INTERVAL_MS = 10
p.ctrl.max_torque  = 1.0;      % [CODE] main.cpp:63   max_torque = 1.0
% [CODE] main.cpp:64  torque_normalization_factor = 1/max_torque —— 恒等于 1，
%        即 calculate_wheel_torque() 的"归一化"是空操作，力矩限幅实际无效。
p.ctrl.torque_norm = 1.0 / p.ctrl.max_torque;
p.ctrl.max_velocity = 2.0;     % [CODE] main.cpp:65   max_velocity = 2.0
% 注意: 该量名义单位 m/s，但 enc_get_speed 返回 rad/s（见 3.7 节说明）

%% ================= 2. 双环 PID（"第六课"经验值）=================
p.pid.vel_kp     = -0.05;      % [CODE] main.cpp:57   速度环 -> 目标俯仰角
p.pid.balance_kp =  0.15;      % [CODE] main.cpp:58   直立环 P
p.pid.balance_kd = -0.02;      % [CODE] main.cpp:59   直立环 D（负号是对的，见 README）
p.pid.robot_kp   =  0.00;      % [CODE] main.cpp:60   位置环（当前未启用）
p.pid.pitch_lim  = 10.0;       % [CODE] main.cpp:451  目标角度限幅 ±10 deg

%% ================= 3. 姿态传感器（单位: 度 / 度每秒）=================
p.sens.pitch_bias  = 0.6;      % [CODE] 6050_middle.cpp:310  pitch 恒加 +0.6 度
p.sens.gyro_scale  = 1/131;    % [CODE] 6050_middle.cpp:271  G_S = 131 -> 度/秒
p.sens.pitch_lag   = 5e-3;     % [ASSUME] Kalman+SMA 等效一阶滞后 5 ms
p.sens.gyro_lag    = 2e-3;     % [ASSUME] 陀螺仪等效滞后 2 ms
p.sens.pitch_noise = 0.15;     % [ASSUME] 俯仰角噪声 RMS (度)
p.sens.gyro_noise  = 1.50;     % [ASSUME] 陀螺仪噪声 RMS (度/秒)
p.sens.gyro_bias   = 0.30;     % [ASSUME] 校准后残余零偏 (度/秒)

%% ================= 4. 编码器测速（rad/s）=================
p.enc.lpf_alpha = 0.3;         % [CODE] esp_hw_link.cpp:110  speed_lpf_alpha
p.enc.fs        = 100;         % [CODE] enc_update 由 motor_update 驱动, 10ms -> 100 Hz
% 一阶低通的等效连续时间常数: tau = -Ts / ln(1-alpha)
p.enc.tau       = -1/p.enc.fs/log(1 - p.enc.lpf_alpha);   % ≈ 28.0 ms
p.enc.ticks     = 4096;        % [CODE] foc_config.h:28  ANGLE_RESOLUTION

%% ================= 5. 电机 / 执行器 =================
% 实测选型：4015 无刷电机（40mm 外径 × 15mm 高），FOC 驱动，7 对极
% 注意：4015 只是外形尺寸代号，其 Kt / 相电阻 / 减速比决定 tau_max，
%       这些参数用户尚未提供，下面 tau_max 仍为估值，需查规格书或实测。
p.motor.model        = '4015';  % [MEASURED] 电机型号
p.motor.TORQUE_MAX_V = 8.0;    % [CODE] foc_config.h:63  TORQUE_MAX_V
p.motor.VBUS         = 12.0;   % [CODE] foc_config.h:5   VBUS
p.motor.pole_pairs   = 7;      % [CODE] foc_config.h:6   POLAR_PAIRS
p.motor.pwm_hz       = 20000;  % [CODE] foc_config.h:32  PWM_FREQUENCY_HZ
% 力矩归一化量程: Uq = torque[-1,1] * TORQUE_MAX_V   (esp_hw_link.cpp:520)
% 电压 -> 电流 -> 转矩的比例在源码里没有给出（取决于所选电机），
% 因此这里直接以"归一化指令 1.0 对应的单轮峰值转矩"作为唯一未知量 tau_max。
p.motor.tau_max = 0.12;        % [MEASURED] 单轮峰值转矩 (N*m) = 0.12，用户确认(2026-11-06)
p.motor.tau_m   = 12e-3;       % [ASSUME] 电流环 + 机械等效一阶时间常数

%% ================= 6. 机械本体（平面 WIP 倒立摆）=================
% 质量来源：实测《元器件重量表.xlsx》（2026-11）
%   螺丝 161.6 / 舵机+舵盘 206 / 打印件 410.8 / 轴承 131.3 / 电池 157.5
%   杜邦线 62.2 / 轮胎 94.1 / 法兰轴 15.6 / 电机 299.3 / ESP32 10.4
%   电机驱动板 3.2 / DC电源模块 17.8 / MPU6050 1.7 / 总线舵机驱动板 8.7
%   总重 1580.2 g
p.mech.m_total  = 1.5802;      % [MEASURED] 整机总重 (kg)
p.mech.R        = 0.030;       % [MEASURED] 车轮半径 3 cm
p.mech.m_wheel  = 0.0941/2;    % [MEASURED] 单轮质量 (kg) = 轮胎 94.1g / 2
                               %   注：电机 299.3g 视为随腿摆动(定子在腿上)，未计入轮重
p.mech.m_body   = 1.5802 - 0.0941;  % [MEASURED] 摆动部分质量 (kg) = 1.4861
p.mech.l        = 0.13;        % [ASSUME] 质心到轮轴距离 (m)，需实测/建模确认
p.mech.J_body   = 8.4e-3;      % [ASSUME] 机身绕质心转动惯量 ≈ m*l^2/3
p.mech.J_wheel  = 1.0e-4;      % [ASSUME] 单轮转动惯量 (轮胎+转子)
p.mech.g        = 9.81;
% 腿位移 -> 机身质心水平位移 的耦合比（腿质量占整机比例，粗估 0.5）
p.mech.leg_coupling = 0.5;     % [ASSUME]

%% ================= 7. 腿部机构 / 舵机 =================
p.leg.L1 = 70;   p.leg.L2 = 120;   p.leg.L3 = 120;   % [CODE] Kms.h:11-13
p.leg.L4 = 70;   p.leg.L5 = 62;                      % [CODE] Kms.h:14-15
p.leg.stand_x = 40;            % [CODE] main.cpp:43   STAND_FORWARD
p.leg.stand_y = 90;            % [CODE] main.cpp:42   STAND_HEIGHT
p.leg.pulse_per_deg = 7.4;     % [CODE] main.cpp:293  PULSE_PER_DEG
p.leg.off_LF =  130;           % [CODE] main.cpp:294  ID2 左前(大腿)
p.leg.off_LR = -130;           % [CODE] main.cpp:295  ID4 左后(小腿)
p.leg.off_RR =    0;           % [CODE] main.cpp:296  ID1 右前(大腿)
p.leg.off_RF =  -75;           % [CODE] main.cpp:297  ID3 右后(小腿)
p.leg.move_ms  = 250;          % [CODE] main.cpp:435  防倒调用时用的时长
% [CODE] main.cpp:331  set_leg_position_smooth 末尾 vTaskDelay(duration_ms + 100)
%        -> 该函数是"阻塞式"的，在 10ms 控制任务里调用会冻结整个平衡环
p.leg.block_s  = (p.leg.move_ms + 100) / 1000;   % = 0.35 s
% 角度 / 脉冲限幅
p.leg.alpha_lim = [80 200];    % [CODE] main.cpp:277-278
p.leg.beta_lim  = [-20 80];    % [CODE] main.cpp:279-280
p.leg.s0_lim_L  = [0 130];     % [CODE] main.cpp:287-288  左腿
p.leg.s2_lim_L  = [0 120];     % [CODE] main.cpp:289-290  左腿
p.leg.s0_lim_R  = [0  90];     % [CODE] main.cpp:306-307  右腿（注意与左侧不同！）
p.leg.s2_lim_R  = [0 180];     % [CODE] main.cpp:308-309  右腿
p.leg.pulse_lim = [500 2500];  % [CODE] main.cpp:315-322
p.leg.base_pulse = 1500;       % [CODE] main.cpp:299

%% ================= 8. 防倒逻辑（adjust_leg_position_for_balance）=================
p.leg_ctrl.gain      = 2.0;    % [CODE] main.cpp:386  pitch_control_gain (mm/deg)
p.leg_ctrl.max_off   = 20.0;   % [CODE] main.cpp:387  max_leg_offset (mm)
p.leg_ctrl.dead_zone = 1.0;    % [CODE] main.cpp:388  dead_zone (deg)
p.leg_ctrl.trig_mm   = 0.5;    % [CODE] main.cpp:434  触发阈值 |dX| > 0.5 mm

%% ================= 9. 仿真设置 =================
p.sim.h        = 1e-3;         % 积分步长 (s)
p.sim.T        = 15;           % 默认仿真时长 (s)
p.sim.fall_deg = 45;           % 判定跌倒的俯仰角阈值 (度)
p.sim.theta0   = 2.0;          % 初始俯仰角（度，后倒为正）
end
