/**
 * @file esp_hw_link.cpp
 * @brief FOC底层硬件接口 + 电机闭环控制 (力矩/速度/位置环)
 *
 * 改造内容:
 *   [P0] 时间常数修正: 1000006 → 1000000
 *   [P0] SVPWM中点注入: 替代无偏置SPWM, Uq=0时严格50%占空比(不发热)
 *   [P0] 全闭环电角度: 所有模式使用编码器角度×极对数, 无开环推演
 *   [P1] PID防积分饱和: 条件积分(Clamping) + 积分分离带
 *   [P1] 电压制式PID: 输出单位为伏特(V), 按电压比例控PWM
 *   [P2] 位置环速度前馈: 虚拟阻尼 -Kff × cur_speed
 *   [P3] 去除printf+冗余I2C: 编码器每周期只读一次
 *
 * 包含:
 *   - AS5600 编码器 I2C 驱动 (单次读取, 无printf)
 *   - LEDC 三相PWM输出 (SVPWM中点注入)
 *   - PID控制器 (电压制式, 带Anti-windup)
 *   - SVPWM空间矢量调制 (Min-Max Injection)
 *   - 三种控制模式: 力矩/速度/位置维持 (全闭环)
 */

#include "esp_hw_link.h"
#include "foc_config.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include <math.h>
#include <stdio.h>

// ================================================================
// 编码器 (AS5600) 驱动实现
// ================================================================
#define AS5600_ADDR     0x36
#define ANGLE_REG       0x0E

typedef struct {
    i2c_master_dev_handle_t dev_hdl;
    bool is_ready;
    float last_angle, last_time, speed;
    float filtered_speed;        // ★ 低通滤波后速度
    float cached_angle;          // ★ enc_update缓存的角度 (避免重复I2C读)
} encoder_t;

static encoder_t g_enc[2] = {};

bool enc_init(int id, int sda, int scl) {
    if (g_enc[id].is_ready) return true;

    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t bc = {};
    bc.i2c_port = (i2c_port_t)id;
    bc.sda_io_num = (gpio_num_t)sda;
    bc.scl_io_num = (gpio_num_t)scl;
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    bc.flags.enable_internal_pullup = true;

    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return false;

    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = AS5600_ADDR;
    dc.scl_speed_hz = 100000;  // ★ 降低I2C频率: 400kHz→100kHz, 提高AS5600通信稳定性

    if (i2c_master_bus_add_device(bus, &dc, &g_enc[id].dev_hdl) != ESP_OK) return false;

    g_enc[id].is_ready = true;
    g_enc[id].last_time = (float)esp_timer_get_time() / 1000000.0f;  // ✅ 已修正: 1000000
    printf("ENC%d OK (SDA=%d SCL=%d)\n", id, sda, scl);
    return true;
}

float enc_read_raw(int id) {
    if (!g_enc[id].is_ready) return -1.0f;

    uint8_t reg = ANGLE_REG, buf[2];
    if (i2c_master_transmit_receive(g_enc[id].dev_hdl, &reg, 1, buf, 2, 30) != ESP_OK)
        return -1.0f;

    uint16_t raw = (buf[0] << 8) | buf[1];
    return (float)raw * 2.0f * M_PI / 4096.0f;
}

float enc_get_cached_angle(int id) { return g_enc[id].cached_angle; }
float enc_get_speed(int id) { return g_enc[id].filtered_speed; }

void enc_update(int id) {
    float now = (float)esp_timer_get_time() / 1000000.0f;  // ✅ 已修正: 1000000
    float ang = enc_read_raw(id);

    if (ang < 0) {
        // ★ I2C超时: 保守策略 — 保持last_angle不变, 速度缓慢衰减
        //   不用速度外推(避免I2C恢复后角度跳跃导致速度尖峰)
        g_enc[id].cached_angle = g_enc[id].last_angle;
        g_enc[id].last_time = now;
        // 速度缓慢衰减到0, 避免突变
        g_enc[id].filtered_speed *= 0.95f;
        return;
    }

    // ---- 成功读取, 缓存角度 ----
    g_enc[id].cached_angle = ang;

    float dt = now - g_enc[id].last_time;
    float diff = ang - g_enc[id].last_angle;
    if (diff > M_PI) diff -= 2 * M_PI;
    else if (diff < -M_PI) diff += 2 * M_PI;
    if (dt > 1e-5f) g_enc[id].speed = diff / dt;

    // ★ 速度低通滤波: 抑制编码器跳变导致的速度噪声
    const float speed_lpf_alpha = 0.3f;
    g_enc[id].filtered_speed = speed_lpf_alpha * g_enc[id].speed +
                               (1.0f - speed_lpf_alpha) * g_enc[id].filtered_speed;

    g_enc[id].last_angle = ang;
    g_enc[id].last_time = now;
}

// ================================================================
// FOC 电机控制参数 & 常量
// ================================================================
#define MOTOR_MAX_NUM       2
#define PWM_FREQ_HZ         20000
#define PWM_DUTY_MAX        1023

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif
#define _2PI_3     2.09439510239319549231
#define _M_PI_2    1.57079632679489661923

// 工具宏
#define _constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static float normalize_angle(float angle) {
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle <  0)      angle += TWO_PI;
    return angle;
}

static float angle_difference(float target, float current) {
    float diff = target - current;
    while (diff >  M_PI) diff -= TWO_PI;
    while (diff < -M_PI) diff += TWO_PI;
    return diff;
}

// ================================================================
// PID 控制器 (电压制式 + Anti-windup)
// ================================================================
typedef struct {
    float kp, ki, kd;
    float ramp, limit;
    float integral_prev;
    float output_prev;
    float error_prev;
    int64_t timestamp_prev;
    float integral_band;          // ★ 新增: 积分分离带阈值
} pid_ctrl_t;

static void pid_reset(pid_ctrl_t* pid) {
    pid->integral_prev = 0.0f;
    pid->output_prev   = 0.0f;
    pid->error_prev    = 0.0f;
    pid->timestamp_prev = esp_timer_get_time();
}

/**
 * @brief PID计算 - 带条件积分(Anti-windup Clamping)和积分分离带
 *
 * 改进点:
 *   1. 输出单位: 伏特(V), 通过limit参数限制最大电压
 *   2. 梯形积分 (误差梯形法, 比矩形更精确)
 *   3. 积分分离: 误差超过integral_band时暂停积分项累积
 *   4. 条件积分(clamping): 输出被限幅时回退本次积分增量
 */
static float pid_compute(pid_ctrl_t* pid, float error) {
    int64_t now = esp_timer_get_time();
    float dt = (now - pid->timestamp_prev) * 1e-6f;
    if (dt <= 0 || dt > 0.5f) {
        // 异常时间间隔, 重置PID防止突变
        pid_reset(pid);
        return 0.0f;
    }

    // --- 比例项 ---
    float p = pid->kp * error;

    // --- 积分项 (梯形 + 分离带 + clamping) ---
    float i_term = 0.0f;
    float integral = pid->integral_prev;

    // 积分分离: 误差过大时不累积积分
    if (pid->integral_band <= 0.0f || fabsf(error) < pid->integral_band) {
        i_term = pid->ki * dt * 0.5f * (error + pid->error_prev);  // 梯形积分
        integral += i_term;
        // 积分量限幅 (软限幅, 约为limit的1/3~1/2)
        integral = _constrain(integral, -pid->limit * 0.4f, pid->limit * 0.4f);
    }

    // --- 微分项 ---
    float d = pid->kd * (error - pid->error_prev) / dt;

    // --- 总输出 ---
    float output = p + integral + d;
    output = _constrain(output, -pid->limit, pid->limit);

    // ★ Anti-windup Clamping: 输出饱和时回退本次积分增量
    if ((output >=  pid->limit && i_term > 0.0f) ||
        (output <= -pid->limit && i_term < 0.0f)) {
        integral = pid->integral_prev;  // 冻结积分, 不让windup继续累积
    }

    // 斜坡限速 (Ramp rate limiting)
    if (pid->ramp > 0) {
        float rate = (output - pid->output_prev) / dt;
        if (rate >  pid->ramp) output = pid->output_prev + pid->ramp * dt;
        if (rate < -pid->ramp) output = pid->output_prev - pid->ramp * dt;
        output = _constrain(output, -pid->limit, pid->limit);
    }

    pid->integral_prev = integral;
    pid->output_prev   = output;
    pid->error_prev    = error;
    pid->timestamp_prev = now;

    return output;
}

// ================================================================
// 电机状态结构体
// ================================================================
typedef enum {
    MODE_TORQUE = 0,
    MODE_SPEED,
    MODE_POSITION_HOLD
} motor_mode_e;

typedef struct {
    uint8_t     mode;
    uint8_t     initialized;
    float       target_torque;
    float       target_speed;
    float       target_angle;
    float       elec_angle_accum;     // 电角度累积 (已弃用开环推演, 保留兼容)
    float       angle_accum;          // 机械角度连续累积 (rad, 可超±2π)
    float       angle_raw_prev;       // 上次原始角度 (用于检测跳变)
    pid_ctrl_t  speed_pid;
    pid_ctrl_t  angle_pid;
    int         pin_a, pin_b, pin_c;
} motor_state_t;

static motor_state_t g_motor[MOTOR_MAX_NUM] = {};

// ================================================================
// 底层硬件: LEDC三相PWM输出
// ================================================================

static uint8_t g_ledc_timer_inited = 0;

static void ledc_timer_init_once(void) {
    if (g_ledc_timer_inited) return;
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode       = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num        = LEDC_TIMER_0;
    timer_conf.duty_resolution  = LEDC_TIMER_10_BIT;
    timer_conf.freq_hz          = PWM_FREQ_HZ;
    timer_conf.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);
    g_ledc_timer_inited = 1;
    printf("LEDC Timer init: %dHz, 10bit\n", PWM_FREQ_HZ);
}

static void ledc_channel_init(int gpio_num, int ledc_channel) {
    ledc_timer_init_once();
    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num   = (gpio_num_t)gpio_num;
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel    = (ledc_channel_t)ledc_channel;
    ch_conf.intr_type  = LEDC_INTR_DISABLE;
    ch_conf.duty       = 0;
    ch_conf.hpoint     = 0;
    ledc_channel_config(&ch_conf);
}

static void motor_pwm_init(int id, int pinA, int pinB, int pinC) {
    int base_ch = id * 3;
    ledc_channel_init(pinA, base_ch + 0);
    ledc_channel_init(pinB, base_ch + 1);
    ledc_channel_init(pinC, base_ch + 2);
    printf("Motor%d PWM init: A=%d B=%d C=%d (ch %d~%d)\n",
           id, pinA, pinB, pinC, base_ch, base_ch + 2);
}

/**
 * @brief 设置三相PWM占空比 (输入范围 [-1, 1], 中心=0即50%占空比)
 *
 * 转换公式: duty_count = (U_phase + 1.0) * 0.5 * PWM_DUTY_MAX
 *   U_phase = -1 → duty = 0% (完全低)
 *   U_phase =  0 → duty = 50% (中间态)
 *   U_phase = +1 → duty = 100% (完全高)
 */
static void motor_set_duty(int id, float Ua, float Ub, float Uc) {
    int base_ch = id * 3;
    uint32_t da = (uint32_t)(_constrain((Ua + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO) * PWM_DUTY_MAX);
    uint32_t db = (uint32_t)(_constrain((Ub + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO) * PWM_DUTY_MAX);
    uint32_t dc = (uint32_t)(_constrain((Uc + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO) * PWM_DUTY_MAX);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(base_ch + 0), da);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(base_ch + 1), db);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(base_ch + 2), dc);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(base_ch + 0));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(base_ch + 1));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(base_ch + 2));
}

// ================================================================
// SVPWM 空间矢量调制 (Min-Max 中点注入法)
// ================================================================

/**
 * @brief SVPWM中点注入调制
 *
 * 原理: 在三相正弦波上叠加零序分量 V0 = -(Vmax + Vmin)/2
 * 效果:
 *   - Uq=0时 ta=tb=tc=0.5 → 严格50%占空比 → 电机不发热!
 *   - 电压利用率提高15.47% (相比普通SPWM)
 *   - 开关损耗与SPWM相同 (因为零序分量不影响线电压)
 *
 * @param Ua, Ub, Uc  归一化三相电压 [-1, 1]
 * @param ta, tb, tc  输出占空比 [0, 1]
 */
static void svpwm_modulate(float Ua, float Ub, float Uc,
                           float* ta, float* tb, float* tc) {
    // 找到三相最大最小值
    float vmax = fmaxf(fmaxf(Ua, Ub), Uc);
    float vmin = fminf(fminf(Ua, Ub), Uc);

    // 零序偏置 (中点注入): 将波形整体平移, 使其以0为中心
    float vzero = -(vmax + vmin) * 0.5f;

    // 叠加零序后映射到 [0, 1]
    *ta = _constrain((Ua + vzero + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO);
    *tb = _constrain((Ub + vzero + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO);
    *tc = _constrain((Uc + vzero + 1.0f) * 0.5f, MIN_DUTY_RATIO, MAX_DUTY_RATIO);
}

/**
 * @brief FOC变换 + SVPWM输出 (电压制式)
 *
 * 流程:
 *   1. 限制Uq在SVPWM线性区内: |Uq| ≤ VBUS × 0.95 × 0.577 (= √3/3)
 *   2. 归一化: Uq_norm = Uq_volts / (VBUS × 0.5)
 *   3. 逆Park变换 (Id=0纯力矩模式):
 *      Uα = -Uq_norm × sin(θe)
 *      Uβ =  Uq_norm × cos(θe)
 *   4. 逆Clarke变换:
 *      Ua =  Uα
 *      Ub = -½Uα + (√3/2)Uβ
 *      Uc = -½Uα - (√3/2)Uβ
 *   5. SVPWM中点注入 → 三相占空比 → 写入LEDC
 *
 * 关键特性:
 *   - Uq_volts=0 时, Ua=Ub=Uc=0, SVPWM输出ta=tb=tc=0.5 (50%占空比)
 *   - 不会像旧代码那样因无DC偏置导致不对称发热
 */
static void foc_output(int id, float Uq_volts, float theta_e) {
    // SVPWM线性区上限: VBUS × max_duty × (1/√3)
    // 超过此值进入过调制区, 波形畸变, 所以要限幅
    float Uq_max = VBUS * MAX_DUTY_RATIO * 0.57735f;  // 0.57735 = 1/√3
    Uq_volts = _constrain(Uq_volts, -Uq_max, Uq_max);

    // 归一化到 [-0.5, 0.5] 范围 (对应SPWM的Uq幅值基准)
    float Uq_norm = Uq_volts / (VBUS * 0.5f);

    // 逆Park变换 (Id = 0, 纯力矩控制)
    float sin_th = sinf(theta_e);
    float cos_th = cosf(theta_e);
    float Ualpha = -Uq_norm * sin_th;
    float Ubeta  =  Uq_norm * cos_th;

    // 逆Clarke变换
    float Ua =  Ualpha;
    float Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
    float Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

    // SVPWM中点注入调制
    float ta, tb, tc;
    svpwm_modulate(Ua, Ub, Uc, &ta, &tb, &tc);

    // 写入硬件PWM (转换为 [-1, 1] 格式给motor_set_duty)
    motor_set_duty(id, ta * 2.0f - 1.0f, tb * 2.0f - 1.0f, tc * 2.0f - 1.0f);
}

// ================================================================
// 电机初始化
// ================================================================

void motor_foc_init(void) {
    for (int i = 0; i < MOTOR_MAX_NUM; i++) {
        motor_state_t* m = &g_motor[i];
        m->mode = MODE_TORQUE;
        m->initialized = 1;
        m->target_torque = 0.0f;
        m->target_speed  = 0.0f;
        m->target_angle  = 0.0f;
        m->elec_angle_accum = 0.0f;
        m->angle_accum     = 0.0f;
        m->angle_raw_prev   = -1.0f;

        // 速度环PID (电压制式, 单位: V)
        m->speed_pid.kp           = SPEED_KP;
        m->speed_pid.ki           = SPEED_KI;
        m->speed_pid.kd           = SPEED_KD;
        m->speed_pid.ramp         = SPEED_RAMP;
        m->speed_pid.limit        = SPEED_LIMIT;         // 最大电压 6V
        m->speed_pid.integral_band = SPEED_INT_BAND;     // 积分分离带 3 rad/s
        pid_reset(&m->speed_pid);

        // 位置环PID (电压制式, 单位: V)
        m->angle_pid.kp           = ANGLE_KP;
        m->angle_pid.ki           = ANGLE_KI;
        m->angle_pid.kd           = ANGLE_KD;
        m->angle_pid.ramp         = ANGLE_RAMP;
        m->angle_pid.limit        = ANGLE_LIMIT;         // 最大电压 10V
        m->angle_pid.integral_band = ANGLE_INT_BAND;     // 积分分离带 0.5 rad
        pid_reset(&m->angle_pid);

        // 设置引脚 (来自 foc_config.h)
        if (i == 0) {
            m->pin_a = MOTOR_PHASE_A_GPIO_0;
            m->pin_b = MOTOR_PHASE_B_GPIO_0;
            m->pin_c = MOTOR_PHASE_C_GPIO_0;
        } else {
            m->pin_a = MOTOR_PHASE_A_GPIO_1;
            m->pin_b = MOTOR_PHASE_B_GPIO_1;
            m->pin_c = MOTOR_PHASE_C_GPIO_1;
        }

        // 初始化PWM
        motor_pwm_init(i, m->pin_a, m->pin_b, m->pin_c);
    }

    printf("=== FOC Motor Init Done (%d motors) ===\n", MOTOR_MAX_NUM);
    printf("=== Mode: SVPWM, VBUS=%.1fV, Closed-loop elec.angle ===\n", VBUS);
}

// ================================================================
// 用户API: 三种控制模式
// ================================================================

void motor_torque(int id, float torque) {
    if (id >= MOTOR_MAX_NUM || !g_motor[id].initialized) return;
    g_motor[id].mode = MODE_TORQUE;
    g_motor[id].target_torque = _constrain(torque, -1.0f, 1.0f);
}

void motor_speed(int id, float target_speed) {
    if (id >= MOTOR_MAX_NUM || !g_motor[id].initialized) return;
    motor_state_t* m = &g_motor[id];
    m->mode = MODE_SPEED;
    m->target_speed = target_speed;
    pid_reset(&m->speed_pid);
}

void motor_position_hold(int id) {
    if (id >= MOTOR_MAX_NUM || !g_motor[id].initialized) return;
    motor_state_t* m = &g_motor[id];

    // 单次I2C读取+缓存 (复用enc_update, 不额外I2C)
    enc_update(id);
    float raw = enc_get_cached_angle(id);

    m->mode = MODE_POSITION_HOLD;
    m->target_angle = raw;
    m->angle_accum = raw;
    m->angle_raw_prev = raw;
    pid_reset(&m->angle_pid);

    printf("Motor%d position_hold locked at %.3f rad (closed-loop)\n", id, raw);
}

// ================================================================
// 核心控制循环 (全闭环电角度 + SVPWM + 电压制式PID)
// ================================================================

void motor_update(int id) {
    if (id >= MOTOR_MAX_NUM || !g_motor[id].initialized) return;
    motor_state_t* m = &g_motor[id];

    // ---- Step 1: 更新编码器 (单次I2C读取, 无printf) ----
    enc_update(id);
    float raw_angle = enc_get_cached_angle(id);  // ★ 从缓存取, 不触发I2C!
    float cur_speed  = enc_get_speed(id);         // ★ 使用低通滤波后的速度

    // ---- 连续机械角度累积 (处理0/2π跳变) ----
    if (m->angle_raw_prev < 0) {
        m->angle_accum     = raw_angle;
        m->angle_raw_prev  = raw_angle;
    } else {
        float delta = raw_angle - m->angle_raw_prev;
        if (delta >  M_PI) delta -= TWO_PI;
        else if (delta < -M_PI) delta += TWO_PI;
        m->angle_accum    += delta;
        m->angle_raw_prev  = raw_angle;
    }

    // ---- Step 2: 统一闭环电角度 (所有模式共用!) ----
    // ★ 核心改造: 不再用目标速度或PID输出推演电角度
    //   直接从编码器真实位置 × 极对数 得到电角度
    float elec_angle = normalize_angle(m->angle_accum * POLAR_PAIRS + ELEC_ANGLE_OFFSET);

    // ---- Step 3: 根据模式计算Uq电压 (单位: 伏特 V) ----
    float Uq_volts = 0.0f;

    switch (m->mode) {

    case MODE_TORQUE:
        // 力矩模式: 目标归一化[-1,1] → 转换为电压
        Uq_volts = m->target_torque * TORQUE_MAX_V;
        break;

    case MODE_SPEED: {
        // 速度环PID: 误差→Uq电压
        float spd_error = m->target_speed - cur_speed;
        Uq_volts = pid_compute(&m->speed_pid, spd_error);
        break;
    }

    case MODE_POSITION_HOLD: {
        // 位置环PID + 速度前馈阻尼
        float pos_err = angle_difference(m->target_angle, normalize_angle(raw_angle));

        // PID输出 (电压)
        Uq_volts = pid_compute(&m->angle_pid, pos_err);

        // ★ 速度前馈 (虚拟阻尼): 抑制位置锁定时的振荡
        //   当电机往目标位置运动时产生反向阻力, 到位后自动减速
        Uq_volts += (-ANGLE_FF_GAIN) * cur_speed;
        break;
    }

    default:
        break;
    }

    // ---- Step 4: SVPWM变换 + PWM输出 ----
    foc_output(id, Uq_volts, elec_angle);

    // 调试打印 (每500次循环, 约0.5秒)
    static int dbg_cnt[2] = {0, 0};
    if (++dbg_cnt[id] >= 500) {
        dbg_cnt[id] = 0;
        printf("M%d: raw=%.3f acc=%.3f ele=%.2f Uq=%.2fV spd=%.2f mode=%d\n",
               id, raw_angle, m->angle_accum, elec_angle, Uq_volts, cur_speed, m->mode);
    }
}
